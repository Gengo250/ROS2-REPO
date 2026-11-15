"""Frames from the camera render worker (web/digital-twin/src/camera/worker.ts).

Binary reply: uint32 LE header length, JSON header, RGBA pixels, optional RGBA instance codes
(rows bottom-up, WebGL order). Images are returned top-down, RGB, uint8; instance codes as int32
(r | g << 8 | b << 16, 0 = background).
"""
from dataclasses import dataclass
import json
import struct
import numpy as np


@dataclass
class Frame:
    header: dict
    rgb: np.ndarray            # H × W × 3 uint8, top row first
    ids: np.ndarray | None     # H × W int32 instance codes, top row first

    @property
    def robot_id(self):
        return self.header['robot_id']

    @property
    def sim_time(self):
        return self.header['sim_time']

    def instance(self, code):
        return next((i for i in self.header['instances'] if i['code'] == code), None)


def parse(message: bytes) -> Frame:
    (length,) = struct.unpack_from('<I', message, 0)
    header = json.loads(message[4:4+length].decode())
    w, h = header['width'], header['height']
    size = w*h*4
    offset = 4+length
    if len(message) != offset+size*(2 if header.get('has_ids') else 1):
        raise ValueError(f'frame size mismatch: {len(message)} bytes for {w}x{h}')
    rgba = np.frombuffer(message, np.uint8, size, offset).reshape(h, w, 4)
    rgb = np.ascontiguousarray(rgba[::-1, :, :3])
    ids = None
    if header.get('has_ids'):
        raw = np.frombuffer(message, np.uint8, size, offset+size).reshape(h, w, 4)[::-1]
        ids = raw[..., 0].astype(np.int32) | (raw[..., 1].astype(np.int32) << 8) | (raw[..., 2].astype(np.int32) << 16)
    return Frame(header, rgb, ids)
