"""Planar LiDAR by polar rasterization of 2D segments; no rendering.

Each visible segment is converted to the interval of beam indices it subtends from the
sensor, and only those (segment, beam) pairs are intersected. The cost therefore follows
the scene's depth complexity instead of beams × segments. Output follows the Gazebo
gpu_lidar convention the stack was tuned on: +inf beyond range_max (no return), -inf below
range_min (REP 117).
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class ScannerSpec:
    frame_id: str
    samples: int
    angle_min: float
    angle_max: float
    range_min: float
    range_max: float
    rate: float
    mount_x: float = 0.
    mount_y: float = 0.
    mount_z: float = 0.
    mount_yaw: float = 0.
    noise_stddev: float = 0.

    @property
    def increment(self):
        return (self.angle_max-self.angle_min)/(self.samples-1)

    @property
    def circle_samples(self):
        """Beams in a virtual full circle at this increment (361 over 180° → 720)."""
        n = round(2*math.pi/self.increment)
        if abs(n*self.increment-2*math.pi) > 1e-6 or n < self.samples:
            raise ValueError('scanner increment must divide the full circle')
        return n


class SegmentSet:
    """Static segments of one scan plane, with cached midpoints for range culling."""

    def __init__(self, a, b):
        self.a = np.asarray(a, dtype=float).reshape(-1, 2)
        self.b = np.asarray(b, dtype=float).reshape(-1, 2)
        self.mid = (self.a+self.b)/2
        self.half = np.hypot(*(self.b-self.a).T)/2


def scan(spec, sensor_x, sensor_y, sensor_yaw, static, dynamic_a=None, dynamic_b=None, rng=None):
    """Ranges (float32, length spec.samples) seen from a sensor pose in the world frame."""
    keep = np.hypot(static.mid[:, 0]-sensor_x, static.mid[:, 1]-sensor_y)-static.half <= spec.range_max
    a, b = static.a[keep], static.b[keep]
    if dynamic_a is not None and len(dynamic_a):
        a, b = np.vstack([a, dynamic_a]), np.vstack([b, dynamic_b])
    ranges = np.full(spec.samples, np.inf)
    if len(a):
        c, s = math.cos(sensor_yaw), math.sin(sensor_yaw)
        rot = np.array([[c, -s], [s, c]])
        qa = (a-[sensor_x, sensor_y])@rot
        qb = (b-[sensor_x, sensor_y])@rot
        e = qb-qa
        # Keep front faces only: the sensor (origin) must lie on the outward side of the CCW
        # edge, whose outward normal is (ey, -ex). Back faces are always occluded.
        visible = (e[:, 0]*qa[:, 1]-e[:, 1]*qa[:, 0]) > 0
        qa, qb, e = qa[visible], qb[visible], e[visible]
        a0 = np.arctan2(qa[:, 1], qa[:, 0])
        span = np.arctan2(np.sin(np.arctan2(qb[:, 1], qb[:, 0])-a0), np.cos(np.arctan2(qb[:, 1], qb[:, 0])-a0))
        lo = np.where(span >= 0, a0, a0+span)
        inc = spec.increment
        # Beam indices on a virtual full circle, so intervals that wrap through ±pi are kept;
        # indices beyond the real field of view are dropped after the modulo below.
        circle = spec.circle_samples
        k0 = np.ceil((lo-spec.angle_min)/inc-1e-9).astype(np.int64)
        k1 = np.floor((lo+np.abs(span)-spec.angle_min)/inc+1e-9).astype(np.int64)
        counts = np.maximum(k1-k0+1, 0)
        total = int(counts.sum())
        if total:
            seg = np.repeat(np.arange(len(counts)), counts)
            starts = np.cumsum(counts)-counts
            k = k0[seg]+np.arange(total)-starts[seg]
            theta = spec.angle_min+k*inc
            dx, dy = np.cos(theta), np.sin(theta)
            ex, ey = e[seg, 0], e[seg, 1]
            denom = dx*ey-dy*ex
            numer = qa[seg, 0]*ey-qa[seg, 1]*ex
            with np.errstate(divide='ignore', invalid='ignore'):
                t = numer/denom
            idx = np.mod(k, circle)
            good = np.isfinite(t) & (t > 0) & (idx < spec.samples)
            np.minimum.at(ranges, idx[good], t[good])
    if spec.noise_stddev > 0 and rng is not None:
        finite = np.isfinite(ranges)
        ranges[finite] += rng.normal(0., spec.noise_stddev, int(finite.sum()))
    ranges[ranges > spec.range_max] = np.inf
    ranges[ranges < spec.range_min] = -np.inf
    return ranges.astype(np.float32)
