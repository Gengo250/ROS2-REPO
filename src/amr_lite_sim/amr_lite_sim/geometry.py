"""2D geometry for the lite engine: convex polygons, SAT overlap, a static broadphase."""
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def oriented_box(x, y, yaw, length, width, forward_offset=0.):
    """CCW corners of a box whose centre is forward_offset ahead of (x, y) along yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    cx, cy = x+c*forward_offset, y+s*forward_offset
    hx, hy = length/2, width/2
    return np.array([[cx+c*px-s*py, cy+s*px+c*py] for px, py in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))])


def disc(x, y, radius, sides=12):
    a = np.arange(sides)*2*math.pi/sides
    return np.column_stack([x+radius*np.cos(a), y+radius*np.sin(a)])


def _axes(poly):
    edges = np.roll(poly, -1, axis=0)-poly
    return np.column_stack([-edges[:, 1], edges[:, 0]])


def overlap(a, b, margin=0.):
    """Separating-axis test for two convex polygons, inflated by ``margin`` in total."""
    for axes in (_axes(a), _axes(b)):
        for axis in axes:
            n = math.hypot(axis[0], axis[1])
            if n < 1e-12:
                continue
            pa, pb = a@axis/n, b@axis/n
            if pa.max()+margin < pb.min() or pb.max()+margin < pa.min():
                return False
    return True


def circle_overlaps_polygon(x, y, radius, poly, box=None):
    """True when a disc touches a convex CCW polygon; ``box`` (bounds) allows a quick reject."""
    if box is not None and (x+radius < box[0] or x-radius > box[2] or y+radius < box[1] or y-radius > box[3]):
        return False
    pts = poly.tolist()
    inside, best2 = True, math.inf
    for i, (ax, ay) in enumerate(pts):
        bx, by = pts[i+1] if i+1 < len(pts) else pts[0]
        ex, ey = bx-ax, by-ay
        if ex*(y-ay)-ey*(x-ax) < 0:
            inside = False
        length2 = ex*ex+ey*ey or 1e-12
        t = ((x-ax)*ex+(y-ay)*ey)/length2
        t = 0. if t < 0 else 1. if t > 1 else t
        dx, dy = x-ax-t*ex, y-ay-t*ey
        best2 = min(best2, dx*dx+dy*dy)
    return inside or best2 < radius*radius


class StaticIndex:
    """Static convex polygons bucketed on a uniform grid for fast local queries."""

    def __init__(self, polygons, cell=2.):
        self.polygons = list(polygons)
        self.cell = cell
        self.boxes = np.array([box_of(p) for p in self.polygons]) if self.polygons else np.zeros((0, 4))
        self.lo, self.hi = self.boxes[:, :2], self.boxes[:, 2:]
        self.buckets = {}
        for i, (x0, y0, x1, y1) in enumerate(self.boxes.tolist()):
            for cx in range(math.floor(x0/cell), math.floor(x1/cell)+1):
                for cy in range(math.floor(y0/cell), math.floor(y1/cell)+1):
                    self.buckets.setdefault((cx, cy), []).append(i)

    def query_ids(self, lo, hi):
        found = set()
        c = self.cell
        for cx in range(math.floor(lo[0]/c), math.floor(hi[0]/c)+1):
            for cy in range(math.floor(lo[1]/c), math.floor(hi[1]/c)+1):
                found.update(self.buckets.get((cx, cy), ()))
        ids = np.fromiter(found, dtype=np.int64, count=len(found))
        if len(ids):
            b = self.boxes[ids]
            ids = ids[(b[:, 0] <= hi[0]) & (b[:, 2] >= lo[0]) & (b[:, 1] <= hi[1]) & (b[:, 3] >= lo[1])]
        return np.sort(ids)

    def query(self, lo, hi):
        return [self.polygons[i] for i in self.query_ids(lo, hi)]

    def collides(self, poly, margin=0.):
        lo, hi = poly.min(0)-margin, poly.max(0)+margin
        return any(overlap(poly, self.polygons[i], margin) for i in self.query_ids(lo, hi))

    def circle_collides(self, x, y, radius):
        c = self.cell
        for cx in range(math.floor((x-radius)/c), math.floor((x+radius)/c)+1):
            for cy in range(math.floor((y-radius)/c), math.floor((y+radius)/c)+1):
                for i in self.buckets.get((cx, cy), ()):
                    if circle_overlaps_polygon(x, y, radius, self.polygons[i], self.boxes[i]):
                        return True
        return False


def box_of(poly):
    """Axis-aligned bounds (min x, min y, max x, max y) of a polygon."""
    xs, ys = poly[:, 0], poly[:, 1]
    return np.array([xs.min(), ys.min(), xs.max(), ys.max()])


def first_overlap(polys_a, boxes_a, polys_b, boxes_b, margin=0.):
    """(i, j) with the smallest i such that polys_a[i] and polys_b[j] overlap, else None.

    A vectorised bounds test prunes the pairs before the exact separating-axis test.
    """
    if not len(polys_a) or not len(polys_b):
        return None
    a, b = np.asarray(boxes_a), np.asarray(boxes_b)
    touch = ((a[:, None, 0] <= b[None, :, 2]+margin) & (b[None, :, 0] <= a[:, None, 2]+margin) &
             (a[:, None, 1] <= b[None, :, 3]+margin) & (b[None, :, 1] <= a[:, None, 3]+margin))
    for i in np.flatnonzero(touch.any(axis=1)):
        for j in np.flatnonzero(touch[i]):
            if overlap(polys_a[i], polys_b[j], margin):
                return int(i), int(j)
    return None


def polygon_segments(polygons):
    """Edges (a, b) of CCW polygons; the outward normal of a→b is (b - a) rotated by -90°."""
    a, b = [], []
    for poly in polygons:
        a.append(poly)
        b.append(np.roll(poly, -1, axis=0))
    if not a:
        return np.zeros((0, 2)), np.zeros((0, 2))
    return np.vstack(a), np.vstack(b)
