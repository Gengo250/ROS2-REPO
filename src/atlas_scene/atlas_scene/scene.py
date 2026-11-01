"""Load the compiled campus scene and expose its geometry in the ROS world frame.

The scene file is the single source shared by the engine, the Gateway and the browser.
Loading verifies both hashes, so a hand-edited or stale file fails loudly.
"""
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
import numpy as np

SHAPES = ('cube', 'cylinder', 'sphere')
# Layers that are floor paint, lane marks or roof: never collide, never seen by floor scanners.
NON_SOLID_LAYERS = frozenset({'ground', 'markings', 'routes', 'roof'})
# The prototype's coarse "solids" are each object's physical envelope: its vehicles collide
# with them and its sensors detect them (rack blocks, conveyor, tables, stairs, fence...).
# Floor-level scanners therefore see every solid, not only the thin parts drawn inside it.
# Walls are already closed by the enclosure layer.
SENSED_SOLID_TAGS = frozenset({'Estante', 'Palete no chão', 'Esteira de triagem', 'Célula de produção', 'Mesa de inspeção',
                               'Estação de recarga', 'Pilar do mezanino', 'Escada do mezanino', 'Grade de proteção',
                               'Extintor', 'Pilar'})


def default_path():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('atlas_scene'))/'config/atlas_campus.scene.json'


@dataclass(frozen=True)
class Primitives:
    """Static primitives of one shape: columns x, y, z, length, width, height, yaw."""
    shape: str
    layer: np.ndarray
    data: np.ndarray

    @property
    def z_min(self):
        return self.data[:, 2]-self.half_height

    @property
    def z_max(self):
        return self.data[:, 2]+self.half_height

    @property
    def half_height(self):
        # Unit cube and unit cylinder span ±0.5 in height; the unit sphere has radius 1.
        return self.data[:, 5] if self.shape == 'sphere' else self.data[:, 5]/2


class Scene:
    def __init__(self, data):
        self.raw = data
        geometry = {'layers': data['layers'], 'solids': data['solids']}
        digest = hashlib.sha256(json.dumps(geometry, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest()
        if digest != data['metadata']['geometry_sha256']:
            raise ValueError('Scene geometry hash mismatch; recompile src/atlas_scene/compiler/compile.mjs')
        whole = dict(data, metadata={k: v for k, v in data['metadata'].items() if k != 'scene_sha256'})
        if hashlib.sha256(json.dumps(whole, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest() \
                != data['metadata']['scene_sha256']:
            raise ValueError('Scene content hash mismatch; recompile src/atlas_scene/compiler/compile.mjs')
        self.geometry_sha256 = digest
        self.scene_sha256 = data['metadata']['scene_sha256']
        self.primitives = {}
        for shape in SHAPES:
            rows, layers = [], []
            for layer, groups in data['layers'].items():
                values = np.asarray(groups.get(shape, ()), dtype=float).reshape(-1, 13)
                if not len(values):
                    continue
                if np.any(values[:, 11:13] != 0):
                    raise ValueError('Static primitives must be yaw-only')
                rows.append(values[:, [0, 1, 2, 3, 4, 5, 10]])
                layers += [layer]*len(values)
            self.primitives[shape] = Primitives(shape, np.asarray(layers), np.vstack(rows) if rows else np.zeros((0, 7)))
        self.solids = data['solids']

    @classmethod
    def load(cls, path=None):
        return cls(json.loads(Path(path or default_path()).read_text()))

    def section(self, key):
        return self.raw[key]

    def solid_mask(self, prims):
        return ~np.isin(prims.layer, list(NON_SOLID_LAYERS))

    def slice_polygons(self, height, circle_sides=16):
        """Convex 2D outlines of every solid primitive cut by the plane z = height."""
        out = []
        for shape, prims in self.primitives.items():
            keep = self.solid_mask(prims) & (prims.z_min <= height) & (prims.z_max >= height)
            for x, y, z, length, width, _h, yaw in prims.data[keep]:
                if shape == 'cube':
                    out.append(rectangle(x, y, length, width, yaw))
                elif shape == 'cylinder':
                    out.append(circle(x, y, length, circle_sides))
                else:
                    radius = float(np.sqrt(max(length*length-(height-z)**2, 0.)))
                    if radius > 1e-3:
                        out.append(circle(x, y, radius, circle_sides))
        out += [rectangle(s['x'], s['y'], s['length'], s['width'], s['yaw'])
                for s in self.solids if s['tag'] in SENSED_SOLID_TAGS]
        return out

    def band_polygons(self, z_low, z_high, circle_sides=16):
        """Outlines of solid primitives overlapping a height band, plus every coarse solid."""
        out = []
        for shape, prims in self.primitives.items():
            keep = self.solid_mask(prims) & (prims.z_min < z_high) & (prims.z_max > z_low)
            for x, y, z, length, width, _h, yaw in prims.data[keep]:
                out.append(rectangle(x, y, length, width, yaw) if shape == 'cube' else circle(x, y, length, circle_sides))
        out += self.solid_polygons()
        return out

    def solid_polygons(self, tags=None):
        return [rectangle(s['x'], s['y'], s['length'], s['width'], s['yaw'])
                for s in self.solids if tags is None or s['tag'] in tags]


def rasterize(polygons, origin, resolution, shape):
    """Boolean grid (rows = y, columns = x) of cell centres inside any convex CCW polygon."""
    grid = np.zeros(shape, dtype=bool)
    ox, oy = origin
    for poly in polygons:
        lo, hi = poly.min(0), poly.max(0)
        c0, r0 = np.floor((lo-[ox, oy])/resolution).astype(int)
        c1, r1 = np.ceil((hi-[ox, oy])/resolution).astype(int)
        c0, r0 = max(c0, 0), max(r0, 0)
        c1, r1 = min(c1, shape[1]), min(r1, shape[0])
        if c0 >= c1 or r0 >= r1:
            continue
        xs = ox+(np.arange(c0, c1)+.5)*resolution
        ys = oy+(np.arange(r0, r1)+.5)*resolution
        px, py = np.meshgrid(xs, ys)
        inside = np.ones(px.shape, dtype=bool)
        for (ax, ay), (bx, by) in zip(poly, np.roll(poly, -1, axis=0)):
            inside &= (bx-ax)*(py-ay)-(by-ay)*(px-ax) >= 0
        grid[r0:r1, c0:c1] |= inside
    return grid


def rectangle(x, y, length, width, yaw):
    """Counter-clockwise corners of a yawed rectangle centred at (x, y)."""
    c, s = np.cos(yaw), np.sin(yaw)
    hx, hy = length/2, width/2
    local = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
    return local@np.array([[c, s], [-s, c]])+np.array([x, y])


def circle(x, y, radius, sides=16):
    a = np.arange(sides)*2*np.pi/sides
    return np.column_stack([x+radius*np.cos(a), y+radius*np.sin(a)])
