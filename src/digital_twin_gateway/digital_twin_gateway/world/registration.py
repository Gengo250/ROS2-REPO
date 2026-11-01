"""Verified world → map SE(2) registration for the campus map: map = R(yaw)·world + (x, y).

Same convention as the Phase 5 route graph. The record must say it was verified against
the map it names; an unverified record is treated as missing (no map data is drawn).
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Registration:
    id: str
    x: float
    y: float
    yaw: float
    map_sha256: str

    def world_to_map(self, wx, wy):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c*wx-s*wy+self.x, s*wx+c*wy+self.y

    def map_to_world(self, mx, my):
        dx, dy = mx-self.x, my-self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c*dx+s*dy, -s*dx+c*dy

    def world_yaw_to_map(self, yaw):
        return math.atan2(math.sin(yaw+self.yaw), math.cos(yaw+self.yaw))


def load(path):
    if not path or not Path(path).is_file():
        return None
    data = json.loads(Path(path).read_text())
    if data.get('parent_frame') != 'world' or data.get('child_frame') != 'map' or not data.get('verified'):
        return None
    t = data['transform']
    return Registration(data['id'], float(t['x']), float(t['y']), float(t['yaw']), data['map_sha256'])
