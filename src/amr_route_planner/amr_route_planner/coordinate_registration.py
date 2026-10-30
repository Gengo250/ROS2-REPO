"""Verified world → map SE(2) registration, bound to one approved map version.

``map_point = R(yaw) · world_point + (x, y)``. The registration is used to express
layout geometry in the Nav2 ``map`` frame and to render map-frame routes in the
world-frame viewer. It is never broadcast as TF and never seeds AMCL.

Calibration fits the real SLAM occupied cells to the warehouse compiler's static
collision geometry (walls, racks, pillars, fixed objects). Layout geometry is used
only to estimate the rigid transform; no occupancy is generated from it.
"""
from dataclasses import dataclass
import json
import math
import numpy as np
from .graph_model import canonical_hash

SCHEMA_VERSION = 1


class RegistrationError(ValueError):
    """Missing, stale, incompatible or unverified registration."""


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class SE2:
    x: float
    y: float
    yaw: float

    def apply(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c*x-s*y+self.x, s*x+c*y+self.y

    def apply_pose(self, x, y, yaw):
        return (*self.apply(x, y), wrap(yaw+self.yaw))

    def inverse(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return SE2(-(c*self.x+s*self.y), s*self.x-c*self.y, wrap(-self.yaw))

    def compose(self, other):
        """self ∘ other: apply ``other`` first."""
        return SE2(*self.apply(other.x, other.y), wrap(self.yaw+other.yaw))

    def as_dict(self):
        return {'x': self.x, 'y': self.y, 'yaw': self.yaw}


def rect_boundary_distance(points, rects):
    """Distance from each (N,2) point to the nearest boundary of axis-aligned (cx, cy, sx, sy) rectangles."""
    r = np.asarray(rects, dtype=float)
    best = np.full(len(points), np.inf)
    # Chunk rectangles to bound memory for large point sets.
    for start in range(0, len(r), 64):
        chunk = r[start:start+64]
        dx = np.abs(points[:, None, 0]-chunk[None, :, 0])-chunk[None, :, 2]/2
        dy = np.abs(points[:, None, 1]-chunk[None, :, 1])-chunk[None, :, 3]/2
        outside = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
        inside = np.abs(np.minimum(np.maximum(dx, dy), 0))
        best = np.minimum(best, np.min(outside+inside, axis=1))
    return best


def residuals(transform, map_points, landmarks):
    """Residual of each map-frame point after mapping it back into the world."""
    inv = transform.inverse()
    c, s = math.cos(inv.yaw), math.sin(inv.yaw)
    world = np.stack([c*map_points[:, 0]-s*map_points[:, 1]+inv.x, s*map_points[:, 0]+c*map_points[:, 1]+inv.y], 1)
    return rect_boundary_distance(world, landmarks)


def residual_summary(values):
    return {'cells': int(len(values)), 'median_m': float(np.median(values)),
            'p75_m': float(np.percentile(values, 75)), 'p90_m': float(np.percentile(values, 90)),
            'inlier_fraction_0_10': float(np.mean(values < .10)),
            'inlier_fraction_0_20': float(np.mean(values < .20))}


def fit(map_points, landmarks, prior, search_xy=.6, search_yaw=.05, cap=.5):
    """Deterministic robust fit: truncated-mean grid search, then coordinate descent."""
    def score(t):
        return float(np.mean(np.minimum(residuals(t, map_points, landmarks), cap)))
    best = None
    for yaw in np.linspace(-search_yaw, search_yaw, 11):
        for dx in np.linspace(-search_xy, search_xy, 13):
            for dy in np.linspace(-search_xy, search_xy, 13):
                t = SE2(prior.x+dx, prior.y+dy, prior.yaw+yaw)
                value = score(t)
                if best is None or value < best[0]-1e-12:
                    best = (value, t)
    value, t = best
    for step in (.05, .02, .01, .005, .0025):
        improved = True
        while improved:
            improved = False
            for dx, dy, dyaw in ((step, 0, 0), (-step, 0, 0), (0, step, 0), (0, -step, 0),
                                 (0, 0, step/10), (0, 0, -step/10)):
                candidate = SE2(t.x+dx, t.y+dy, t.yaw+dyaw)
                trial = score(candidate)
                if trial < value-1e-12:
                    value, t, improved = trial, candidate, True
    return SE2(round(t.x, 6), round(t.y, 6), round(t.yaw, 7)), value


def evaluate(record, map_version, layout_sha256):
    """Raise RegistrationError unless the record is verified for this exact map and layout."""
    if not isinstance(record, dict) or record.get('schema_version') != SCHEMA_VERSION:
        raise RegistrationError('Unsupported or missing registration record')
    if record.get('parent_frame') != 'world' or record.get('child_frame') != 'map':
        raise RegistrationError('Registration must map world → map')
    if record.get('map_version') != map_version:
        raise RegistrationError('Registration belongs to a different map version; recalibrate after remapping')
    if record.get('layout_sha256') != layout_sha256:
        raise RegistrationError('Registration belongs to a different warehouse layout')
    if not record.get('valid') or record.get('reasons'):
        raise RegistrationError('Registration failed acceptance: '+'; '.join(record.get('reasons') or ['not valid']))
    t = record.get('transform', {})
    if not all(isinstance(t.get(k), (int, float)) and math.isfinite(t[k]) for k in ('x', 'y', 'yaw')):
        raise RegistrationError('Registration transform must be finite')
    if record.get('id') != registration_id(record):
        raise RegistrationError('Registration identifier does not match its content')
    transform = SE2(t['x'], t['y'], t['yaw'])
    # Round-trip check guards against a malformed inverse implementation or data.
    for probe in ((0., 0.), (10., -5.), (-14., 9.)):
        back = transform.inverse().apply(*transform.apply(*probe))
        if math.dist(back, probe) > 1e-9:
            raise RegistrationError('Registration inverse is inconsistent')
    return transform


def registration_id(record):
    return canonical_hash({k: v for k, v in record.items() if k != 'id'})[:16]


def accept(record, thresholds):
    """Fill ``valid``/``reasons`` from measured residuals and prior agreement."""
    reasons = []
    r = record['residuals']
    if r['cells'] < thresholds['min_cells']:
        reasons.append(f"only {r['cells']} occupied cells")
    if r['median_m'] > thresholds['max_median_m']:
        reasons.append(f"median residual {r['median_m']:.3f} m > {thresholds['max_median_m']} m")
    if r['inlier_fraction_0_20'] < thresholds['min_inlier_fraction_0_20']:
        reasons.append(f"inlier fraction {r['inlier_fraction_0_20']:.2f} < {thresholds['min_inlier_fraction_0_20']}")
    t, p = record['transform'], record['prior']['transform']
    if math.hypot(t['x']-p['x'], t['y']-p['y']) > thresholds['max_prior_translation_m']:
        reasons.append('fit disagrees with the controlled initial-pose prior (translation)')
    if abs(wrap(t['yaw']-p['yaw'])) > thresholds['max_prior_yaw_rad']:
        reasons.append('fit disagrees with the controlled initial-pose prior (yaw)')
    record['valid'] = not reasons
    record['reasons'] = reasons
    record['id'] = registration_id(record)
    return record


def load(path, map_version, layout_sha256):
    try:
        record = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise RegistrationError(f'Registration unavailable: {path}') from exc
    return record, evaluate(record, map_version, layout_sha256)


def active(paths=None):
    """The verified registration of the installed approved map and layout, for evaluation tools.

    Raises RegistrationError (never falls back to a fixed offset) when the record does
    not belong to the active map version and warehouse layout.
    """
    from . import graph_builder as gb
    from . import map_validation as mv
    paths = paths or gb.Paths()
    try:
        record = json.loads(paths.file('registration').read_text())
        version = mv.OccupancyMap.from_yaml(paths.file('map_yaml')).version()
        layout = gb.digest(paths.file('layout'))
    except (OSError, ValueError) as exc:
        raise RegistrationError(f'Active registration unavailable: {exc}') from exc
    return evaluate(record, version, layout), record
