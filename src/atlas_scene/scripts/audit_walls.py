#!/usr/bin/env python3
"""Coverage audit of the four exterior walls of a compiled campus scene.

Samples each wall plane on a 0.25 m (along the wall) x 0.25 m (height) grid from the floor to
the roof line and reports which fraction is covered by drawn geometry, for a set of layers
(e.g. the browser's default before/after the building shell). Dock door openings are reported
separately so that a closed door is not counted as a wall gap. Read-only; prints JSON.

    python3 audit_walls.py scene.json --layers architecture
    python3 audit_walls.py scene.json --layers architecture enclosure walls_west walls_east walls_north walls_south
"""
import argparse
import json
import numpy as np

# Wall planes (world frame): constant coordinate, sampled span, plane offset inside the wall.
PLANES = {'west': ('x', -73.0, (-54., 54.)), 'east': ('x', 73.0, (-54., 54.)),
          'north': ('y', 54.0, (-73.35, 73.35)), 'south': ('y', -53.95, (-73.35, 73.35))}
DOORS = [-56+16*i for i in range(8)]


def primitives(scene, layers):
    boxes, cylinders = [], []
    for layer in layers:
        for shape, values in scene['layers'].get(layer, {}).items():
            v = np.asarray(values, dtype=float).reshape(-1, 13)
            if shape == 'cube':
                boxes.append(v[:, [0, 1, 2, 3, 4, 5, 10]])
            elif shape == 'cylinder':
                cylinders.append(v[:, [0, 1, 2, 3, 5]])
    return (np.vstack(boxes) if boxes else np.zeros((0, 7))), (np.vstack(cylinders) if cylinders else np.zeros((0, 5)))


def covered(boxes, cylinders, px, py, pz):
    """Boolean array: points (px, py, pz) inside any (yawed) box or vertical cylinder."""
    hit = np.zeros(px.shape, dtype=bool)
    for x, y, z, length, width, height, yaw in boxes:
        c, s = np.cos(yaw), np.sin(yaw)
        lx, ly = c*(px-x)+s*(py-y), -s*(px-x)+c*(py-y)
        hit |= (abs(lx) <= length/2+1e-6) & (abs(ly) <= width/2+1e-6) & (abs(pz-z) <= height/2+1e-6)
    for x, y, z, radius, height in cylinders:
        hit |= ((px-x)**2+(py-y)**2 <= radius**2) & (abs(pz-z) <= height/2)
    return hit


def audit(scene, layers, top=14.0, step=.25):
    boxes, cylinders = primitives(scene, layers)
    out = {}
    for side, (axis, value, (a, b)) in PLANES.items():
        s = np.arange(a+step/2, b, step)
        z = np.arange(step/2, top, step)
        S, Z = np.meshgrid(s, z)
        px, py = (np.full_like(S, value), S) if axis == 'x' else (S, np.full_like(S, value))
        hit = covered(boxes, cylinders, px, py, Z)
        door = np.zeros_like(hit) if side != 'south' else \
            (np.min([abs(S-d) for d in DOORS], axis=0) < 4.2) & (Z < 5.2)
        wall = ~door
        rows = {}
        for lo, hi in ((0, .5), (.5, 2), (2, 3), (3, 6), (6, 10), (10, top)):
            band = wall & (Z >= lo) & (Z < hi)
            rows[f'{lo:g}-{hi:g} m'] = round(float(hit[band].mean()), 4) if band.any() else None
        # Largest uncovered run along the wall at robot/person height (0.1-2 m).
        low = ~hit[(z >= .1) & (z < 2)].any(axis=0) & ~door[(z >= .1) & (z < 2)].any(axis=0)
        runs, run = [], 0
        for flag in low:
            run = run+1 if flag else 0
            runs.append(run)
        out[side] = dict(coverage_excluding_doors=round(float(hit[wall].mean()), 4), by_height=rows,
                         largest_open_run_below_2m_m=round(max(runs, default=0)*step, 2),
                         door_openings_covered=round(float(hit[door].mean()), 4) if door.any() else None)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('scene')
    p.add_argument('--layers', nargs='+', required=True)
    a = p.parse_args()
    scene = json.load(open(a.scene))
    print(json.dumps({'scene_sha256': scene['metadata']['scene_sha256'], 'layers': a.layers,
                      'walls': audit(scene, a.layers)}, indent=1))


if __name__ == '__main__':
    main()
