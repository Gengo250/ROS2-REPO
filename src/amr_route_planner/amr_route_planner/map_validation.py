"""Occupancy-map and robot-footprint traversability checks in the Nav2 map frame.

Unknown cells are never treated as free. A node or edge is ``validated`` only when
every cell under the padded Nav2 footprint, swept along the edge and rotated in
place at its ends, plus the required clearance band, is known free.
"""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from .graph_model import canonical_hash

FREE, OCCUPIED, UNKNOWN = 0, 100, -1


class MapError(ValueError):
    """Map missing, malformed or inconsistent with the approved version."""


@dataclass(frozen=True)
class Footprint:
    """Padded rectangular Nav2 footprint relative to base_link (metres)."""
    front: float
    back: float
    half_width: float

    @property
    def circumscribed(self):
        return math.hypot(max(self.front, self.back), self.half_width)

    @classmethod
    def from_nav2(cls, params):
        costmap = params['global_costmap']['global_costmap']['ros__parameters']
        points = yaml.safe_load(costmap['footprint']) if isinstance(costmap['footprint'], str) else costmap['footprint']
        pad = float(costmap.get('footprint_padding', 0.))
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        return cls(max(xs)+pad, -min(xs)+pad, max(abs(v) for v in ys)+pad)


class OccupancyMap:
    """Row 0 is the bottom row, matching nav_msgs/OccupancyGrid."""

    def __init__(self, grid, resolution, origin):
        self.grid = np.asarray(grid, dtype=np.int8)
        self.resolution, self.origin = float(resolution), tuple(float(v) for v in origin)
        if self.grid.ndim != 2 or self.resolution <= 0 or not all(math.isfinite(v) for v in self.origin):
            raise MapError('Invalid occupancy grid geometry')
        if abs(self.origin[2]) > 1e-9:
            raise MapError('Rotated map origins are not supported')

    @classmethod
    def from_yaml(cls, path):
        path = Path(path)
        try:
            meta = yaml.safe_load(path.read_text())
            image = np.array(Image.open(path.parent/meta['image']).convert('L'), dtype=float)
        except (OSError, KeyError, yaml.YAMLError) as exc:
            raise MapError(f'Cannot load map {path}') from exc
        if meta.get('mode', 'trinary') != 'trinary':
            raise MapError('Only trinary maps are supported')
        # nav2_map_server trinary interpretation.
        p = image/255. if meta.get('negate', 0) else (255.-image)/255.
        grid = np.full(image.shape, UNKNOWN, dtype=np.int8)
        grid[p > meta['occupied_thresh']] = OCCUPIED
        grid[p < meta['free_thresh']] = FREE
        return cls(grid[::-1], meta['resolution'], meta['origin'])

    @classmethod
    def from_message(cls, msg):
        info = msg.info
        q = info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        return cls(grid, info.resolution, (info.origin.position.x, info.origin.position.y, yaw))

    @property
    def shape(self):
        return self.grid.shape

    def version(self):
        """Content identity of what Nav2 actually loads: geometry plus cell states."""
        # 6 decimals: OccupancyGrid carries float32 resolution (0.05 → 0.0500000007) and
        # map YAML geometry is specified to millimetres, so both loaders must agree.
        return canonical_hash({'resolution': round(self.resolution, 6), 'origin': [round(v, 6)+0. for v in self.origin],
                               'width': self.grid.shape[1], 'height': self.grid.shape[0],
                               'data_sha256': hashlib.sha256(np.ascontiguousarray(self.grid).tobytes()).hexdigest()})

    def region_counts(self, shapes, exclude=()):
        """Count free/occupied/unknown cells covered by any shape (minus ``exclude``), plus area outside the map."""
        xmin = min(s.bounds()[0] for s in shapes); ymin = min(s.bounds()[1] for s in shapes)
        xmax = max(s.bounds()[2] for s in shapes); ymax = max(s.bounds()[3] for s in shapes)
        res, (ox, oy, _) = self.resolution, self.origin
        h, w = self.grid.shape
        i0, i1 = math.floor((xmin-ox)/res), math.ceil((xmax-ox)/res)
        j0, j1 = math.floor((ymin-oy)/res), math.ceil((ymax-oy)/res)
        ii, jj = np.meshgrid(np.arange(i0, i1+1), np.arange(j0, j1+1))
        cx, cy = ox+(ii+.5)*res, oy+(jj+.5)*res
        # Half-diagonal margin: any cell overlapping the shape counts, not only its centre.
        margin = res*math.sqrt(2)/2
        mask = np.zeros(ii.shape, dtype=bool)
        for shape in shapes:
            mask |= shape.contains(cx, cy, margin)
        for shape in exclude:
            mask &= ~shape.contains(cx, cy, margin)
        inside_map = (ii >= 0) & (ii < w) & (jj >= 0) & (jj < h)
        values = self.grid[jj[mask & inside_map], ii[mask & inside_map]]
        return {'free': int(np.sum(values == FREE)), 'occupied': int(np.sum(values == OCCUPIED)),
                'unknown': int(np.sum((values != FREE) & (values != OCCUPIED))),
                'outside': int(np.sum(mask & ~inside_map))}

    def occupied_points(self):
        jj, ii = np.nonzero(self.grid == OCCUPIED)
        return np.stack([self.origin[0]+(ii+.5)*self.resolution, self.origin[1]+(jj+.5)*self.resolution], 1)


@dataclass(frozen=True)
class Box:
    """Oriented rectangle: centre, heading of the long axis, half extents."""
    cx: float
    cy: float
    yaw: float
    half_length: float
    half_width: float

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [(self.cx+c*u-s*v, self.cy+s*u+c*v) for u, v in
                ((-self.half_length, -self.half_width), (self.half_length, -self.half_width),
                 (self.half_length, self.half_width), (-self.half_length, self.half_width))]

    def bounds(self):
        xs, ys = zip(*self.corners())
        return min(xs), min(ys), max(xs), max(ys)

    def contains(self, x, y, margin=0.):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        u, v = c*(x-self.cx)+s*(y-self.cy), -s*(x-self.cx)+c*(y-self.cy)
        return (np.abs(u) <= self.half_length+margin) & (np.abs(v) <= self.half_width+margin)


@dataclass(frozen=True)
class Disc:
    cx: float
    cy: float
    radius: float

    def bounds(self):
        return self.cx-self.radius, self.cy-self.radius, self.cx+self.radius, self.cy+self.radius

    def contains(self, x, y, margin=0.):
        return np.hypot(x-self.cx, y-self.cy) <= self.radius+margin


def sweep(start, end, footprint, clearance=0.):
    """Area covered while driving straight from ``start`` to ``end`` facing along the segment."""
    length = math.dist(start, end)
    yaw = math.atan2(end[1]-start[1], end[0]-start[0]) if length > 1e-9 else 0.
    c, s = math.cos(yaw), math.sin(yaw)
    lo, hi = -footprint.back-clearance, length+footprint.front+clearance
    mid = (lo+hi)/2
    return Box(start[0]+c*mid, start[1]+s*mid, yaw, (hi-lo)/2, footprint.half_width+clearance)


def rotation(point, footprint, clearance=0.):
    """Area covered by rotating in place (the route may reverse at any node)."""
    return Disc(point[0], point[1], footprint.circumscribed+clearance)


def classify(occupancy, core_shapes, clearance_shapes):
    """Classify one element. ``core`` = padded footprint; ``clearance`` adds the required margin."""
    core, full = occupancy.region_counts(core_shapes), occupancy.region_counts(clearance_shapes)
    detail = {'core': core, 'with_clearance': full}
    if full['outside']:
        return 'unmapped', 'outside the occupancy map', detail
    if core['occupied']:
        return 'blocked', 'occupied map cells under the robot footprint', detail
    if full['occupied']:
        return 'blocked', 'occupied map cells inside the required clearance', detail
    if full['unknown']:
        return 'unmapped', 'unknown map cells under the footprint or clearance band', detail
    return 'validated', 'known free footprint and clearance', detail


def node_shapes(point, footprint, clearance):
    return [rotation(point, footprint)], [rotation(point, footprint, clearance)]


def edge_shapes(start, end, footprint, clearance):
    return ([sweep(start, end, footprint), rotation(start, footprint), rotation(end, footprint)],
            [sweep(start, end, footprint, clearance), rotation(start, footprint, clearance),
             rotation(end, footprint, clearance)])


def polygons_intersect(a, b):
    """Separating-axis test for two convex polygons."""
    for poly in (a, b):
        for (x0, y0), (x1, y1) in zip(poly, poly[1:]+poly[:1]):
            nx, ny = y0-y1, x1-x0
            pa = [nx*x+ny*y for x, y in a]
            pb = [nx*x+ny*y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def rect_polygon(cx, cy, sx, sy):
    return [(cx-sx/2, cy-sy/2), (cx+sx/2, cy-sy/2), (cx+sx/2, cy+sy/2), (cx-sx/2, cy+sy/2)]


def shape_hits_rect(shape, rect):
    cx, cy, sx, sy = rect
    if isinstance(shape, Disc):
        dx, dy = max(abs(shape.cx-cx)-sx/2, 0.), max(abs(shape.cy-cy)-sy/2, 0.)
        return math.hypot(dx, dy) < shape.radius
    return polygons_intersect(shape.corners(), rect_polygon(cx, cy, sx, sy))
