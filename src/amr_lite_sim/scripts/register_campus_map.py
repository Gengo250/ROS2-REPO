#!/usr/bin/env python3
"""World → map registration of a campus SLAM map, verified against the scene geometry.

SLAM anchors its map at the first scan, i.e. at the robot's spawn pose, so the candidate
transform is the inverse spawn pose. It is accepted only if the map agrees with the scene:
  * occupied cells lie on scene geometry (median and inlier fraction within 0.10 m);
  * the scene's walls/solids inside the surveyed region are observed (coverage);
  * no map free cell lies inside a solid (free-in-solid fraction);
  * each enclosure wall present at the scan plane is mapped along its inner face (enclosure
    fraction per wall; a face sample outside the map image counts as missing).
Writes map = R(yaw)·world + (x, y) with the evidence; the Gateway refuses unverified records.
Usage: register_campus_map.py --map campus.yaml --output campus_registration.json [--height 0.48]
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import yaml
from scipy.ndimage import binary_erosion, distance_transform_edt
from atlas_scene.scene import Scene, rasterize

LIMITS = {'median_m': 0.05, 'inlier_fraction_0_10': 0.95, 'free_in_solid_fraction': 0.005, 'coverage_fraction': 0.80,
          'enclosure_fraction_min': 0.80}


def read_pgm(path):
    """Binary PGM (P5) as a uint8 array; comment lines in the header are skipped."""
    data = Path(path).read_bytes()
    header, i = [], 0
    while len(header) < 4:
        j = data.index(b'\n', i)
        line, i = data[i:j], j+1
        if not line.startswith(b'#'):
            header += line.split()
    width, height = int(header[1]), int(header[2])
    return np.frombuffer(data[i:i+width*height], dtype=np.uint8).reshape(height, width)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--map', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--scene', default='')
    p.add_argument('--spawn', default='amr_01_spawn')
    p.add_argument('--height', type=float, default=0.48, help='scan plane of the mapping scanner')
    a = p.parse_args()
    scene = Scene.load(a.scene or None)
    meta = yaml.safe_load(a.map.read_text())
    image = read_pgm(a.map.parent/meta['image'])
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    spawn = next(s for s in scene.section('spawns') if s['id'] == a.spawn)['pose']
    sx, sy, syaw = spawn[0], spawn[1], spawn[5]
    yaw = -syaw
    c, s = math.cos(yaw), math.sin(yaw)
    tx, ty = -(c*sx-s*sy), -(s*sx+c*sy)

    def map_to_world(mx, my):
        dx, dy = mx-tx, my-ty
        return c*dx+s*dy, -s*dx+c*dy
    rows, cols = image.shape
    row_index, col_index = np.mgrid[0:rows, 0:cols]
    mx = ox+(col_index+.5)*res
    my = oy+(rows-1-row_index+.5)*res
    wx, wy = map_to_world(mx, my)
    occupied = image < 50           # trinary map: 0 occupied, 254 free, 205 unknown
    free = image > 250
    # Scene geometry at the scanner plane, 2 cm grid in the world frame.
    g, gx0, gy0 = 0.02, -80., -60.
    shape = (int(120/g), int(160/g))
    solid = rasterize(scene.slice_polygons(a.height), (gx0, gy0), g, shape)
    distance = distance_transform_edt(~solid)*g

    def sample(field, x, y):
        cc = np.clip(((x-gx0)/g).astype(int), 0, shape[1]-1)
        rr = np.clip(((y-gy0)/g).astype(int), 0, shape[0]-1)
        return field[rr, cc]
    d = sample(distance, wx[occupied], wy[occupied])
    inside_solid = sample(binary_erosion(solid, iterations=3), wx[free], wy[free])
    # Coverage: scene boundary cells (inside the building) near any observed free cell must be occupied.
    boundary = solid & ~binary_erosion(solid)
    by, bx = np.nonzero(boundary)
    bwx, bwy = gx0+(bx+.5)*g, gy0+(by+.5)*g
    i = scene.section('building')['interior']
    keep = (bwx > i['x_min']) & (bwx < i['x_max']) & (bwy > i['y_min']) & (bwy < i['y_max'])
    bwx, bwy = bwx[keep][::5], bwy[keep][::5]
    cm, sm = math.cos(yaw), math.sin(yaw)
    bmx, bmy = cm*bwx-sm*bwy+tx, sm*bwx+cm*bwy+ty
    bc = ((bmx-ox)/res).astype(int)
    br = rows-1-((bmy-oy)/res).astype(int)
    valid = (bc >= 1) & (bc < cols-1) & (br >= 1) & (br < rows-1)
    known = np.zeros(valid.shape, dtype=bool)
    seen = np.zeros(valid.shape, dtype=bool)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            cell = image[np.clip(br+dr, 0, rows-1), np.clip(bc+dc, 0, cols-1)]
            known |= valid & (cell != 205)
            seen |= valid & (cell < 50)
    # Enclosure: the boundary test above only sees cells inside the interior and inside the image,
    # so a wall lost at the edge of the SLAM grid would pass it. Sample each inner face instead.
    occupied_distance = distance_transform_edt(~occupied)*res
    enclosure = {}
    for name, along, fixed, nx, ny in (('north', 'x', i['y_max'], 0, 1), ('south', 'x', i['y_min'], 0, -1),
                                       ('west', 'y', i['x_min'], -1, 0), ('east', 'y', i['x_max'], 1, 0)):
        t = np.arange(i[f'{along}_min'], i[f'{along}_max'], .25)
        fx, fy = (t, np.full_like(t, fixed)) if along == 'x' else (np.full_like(t, fixed), t)
        # First scene surface along the outward normal within ±0.6 m of the interior bound
        # (wall, door panel or dock equipment at the scan plane); none = an opening.
        steps = np.arange(-.6, .6, .02)
        hits = np.stack([sample(solid, fx+nx*k, fy+ny*k) for k in steps])
        wall = hits.any(0)
        face = steps[hits.argmax(0)]
        px, py = fx+nx*(face-.01), fy+ny*(face-.01)
        pc = ((cm*px-sm*py+tx-ox)/res).astype(int)
        pr = rows-1-((sm*px+cm*py+ty-oy)/res).astype(int)
        inside = (pc >= 0) & (pc < cols) & (pr >= 0) & (pr < rows)
        mapped = np.zeros(len(t), dtype=bool)
        mapped[inside] = occupied_distance[pr[inside], pc[inside]] <= .10
        enclosure[name] = float(mapped[wall].mean()) if wall.any() else 1.
    stats = {'occupied_cells': int(occupied.sum()), 'median_m': float(np.median(d)), 'p90_m': float(np.percentile(d, 90)),
             'inlier_fraction_0_10': float((d <= .10).mean()), 'free_cells': int(free.sum()),
             'free_in_solid_fraction': float(inside_solid.mean()),
             'coverage_fraction': float(seen[known].mean()) if known.any() else 0., 'boundary_samples_known': int(known.sum()),
             'enclosure_fraction': enclosure, 'enclosure_fraction_min': min(enclosure.values())}
    checks = {'median_m': stats['median_m'] <= LIMITS['median_m'],
              'inlier_fraction_0_10': stats['inlier_fraction_0_10'] >= LIMITS['inlier_fraction_0_10'],
              'free_in_solid_fraction': stats['free_in_solid_fraction'] <= LIMITS['free_in_solid_fraction'],
              'coverage_fraction': stats['coverage_fraction'] >= LIMITS['coverage_fraction'],
              'enclosure_fraction_min': stats['enclosure_fraction_min'] >= LIMITS['enclosure_fraction_min']}
    image_bytes = (a.map.parent/meta['image']).read_bytes()
    record = {'id': hashlib.sha256(f'{tx:.6f}{ty:.6f}{yaw:.6f}'.encode()+image_bytes).hexdigest()[:16],
              'parent_frame': 'world', 'child_frame': 'map', 'convention': 'map = R(yaw)*world + (x, y)',
              'transform': {'x': tx, 'y': ty, 'yaw': yaw}, 'method': f'inverse of spawn pose {a.spawn} (SLAM map anchored at the first scan), '
              'verified against the scene geometry at the scanner plane', 'scan_plane_height_m': a.height,
              'verified': all(checks.values()), 'checks': checks, 'limits': LIMITS, 'residuals': stats,
              'map_sha256': hashlib.sha256(image_bytes).hexdigest(), 'scene_sha256': scene.scene_sha256,
              'created_at': datetime.now(timezone.utc).isoformat()}
    a.output.write_text(json.dumps(record, indent=1)+'\n')
    print(json.dumps({'verified': record['verified'], **stats}))


if __name__ == '__main__':
    main()
