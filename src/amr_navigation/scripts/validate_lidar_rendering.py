#!/usr/bin/env python3
"""Simulated-LiDAR integrity: do /scan and /top_scan report the world geometry?

Owns one mapping-only simulation (no workers, so the scene is static). The AMR is
placed at fixed poses; each recorded beam is compared with a ray cast against the
warehouse model's visual boxes and vertical cylinders (what gpu_lidar renders).
Gazebo poses and model geometry are evaluation data only; nothing is fed back.

A beam is ``missing`` when geometry lies within range but the sensor returned no
hit, and ``mismatch`` when both returned but differ by more than AGREE_M. The
renderer/driver fault found in Phase 5.1 shows up as whole sectors of missing beams.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

AGREE_M = 0.10
MAX_MISSING_FRACTION = 0.02
POSES = {'spawn': [-11.0, 0.0, 0.015, 0, 0, 0.0], 'junction_east': [-4.5, 0.0, 0.015, 0, 0, 0.0],
         'junction_north': [-4.5, 0.0, 0.015, 0, 0, math.pi/2], 'west_service_north': [-8.7, 5.4, 0.015, 0, 0, math.pi/2]}


def pose(text):
    v = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    return v+[0.]*(6-len(v))


def compose(a, b):
    c, s = math.cos(a[5]), math.sin(a[5])
    return [a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2], 0, 0, a[5]+b[5]]


def shapes(models, model, frame, out):
    """Visual boxes and upright cylinders of ``model`` (includes resolved) in world coordinates."""
    root = ET.parse(models/model/'model.sdf').getroot().find('model')
    for link in root.findall('link'):
        lf = compose(frame, pose(link.findtext('pose')))
        for vis in link.findall('visual'):
            p = pose(vis.findtext('pose'))
            box, cyl = vis.find('geometry/box/size'), vis.find('geometry/cylinder')
            upright = abs(p[3]) < 1e-6 and abs(p[4]) < 1e-6
            if box is not None and upright:
                out.append(('box', compose(lf, p), [float(x) for x in box.text.split()]))
            elif cyl is not None and upright:
                out.append(('cylinder', compose(lf, p), [float(cyl.findtext('radius')), float(cyl.findtext('length'))]))
            else:
                out.append(('unmodelled', compose(lf, p), vis.get('name')))
    for inc in root.findall('include'):
        shapes(models, inc.findtext('uri').replace('model://', ''), compose(frame, pose(inc.findtext('pose'))), out)


def ray_cast(ox, oy, oz, dx, dy, items, rmax):
    best = rmax
    for kind, (x, y, z, _, _, yaw), size in items:
        if kind == 'box':
            sx, sy, sz = size
            if not z-sz/2 <= oz <= z+sz/2:
                continue
            c, s = math.cos(-yaw), math.sin(-yaw)
            px, py, qx, qy = c*(ox-x)-s*(oy-y), s*(ox-x)+c*(oy-y), c*dx-s*dy, s*dx+c*dy
            t0, t1 = 0., best
            for p, q, h in ((px, qx, sx/2), (py, qy, sy/2)):
                if abs(q) < 1e-12:
                    t0 = math.inf if abs(p) > h else t0
                    continue
                a, b = (-h-p)/q, (h-p)/q
                t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
            if t0 <= t1:
                best = min(best, t0)
        elif kind == 'cylinder':
            r, length = size
            if not z-length/2 <= oz <= z+length/2:
                continue
            fx, fy = ox-x, oy-y
            b, cc = fx*dx+fy*dy, fx*fx+fy*fy-r*r
            disc = b*b-cc
            if disc >= 0 and -b-math.sqrt(disc) >= 0:
                best = min(best, -b-math.sqrt(disc))
    return best


def compare(msg, origin, yaw, items):
    """Per-beam classification and per-45-degree-sector missing counts."""
    ox, oy, oz = origin
    counts = {'agree': 0, 'no_hit_expected_and_measured': 0, 'missing': 0, 'mismatch': 0}
    sectors, errors = {}, []
    near = [i for i in items if i[0] != 'unmodelled' and math.hypot(i[1][0]-ox, i[1][1]-oy) < msg.range_max+3]
    for j, r in enumerate(msg.ranges):
        rel = msg.angle_min+j*msg.angle_increment
        a = yaw+rel
        expected = ray_cast(ox, oy, oz, math.cos(a), math.sin(a), near, msg.range_max)
        measured = r if math.isfinite(r) and r <= msg.range_max else msg.range_max
        sector = int(((math.degrees(rel)+180) % 360)//45)*45-180
        bucket = sectors.setdefault(sector, {'beams': 0, 'missing': 0})
        bucket['beams'] += 1
        if expected >= msg.range_max and measured >= msg.range_max:
            counts['no_hit_expected_and_measured'] += 1
        elif abs(measured-expected) <= AGREE_M:
            counts['agree'] += 1
            errors.append(abs(measured-expected))
        elif measured >= msg.range_max:
            counts['missing'] += 1
            bucket['missing'] += 1
        else:
            counts['mismatch'] += 1
    errors.sort()
    counts['median_agree_error_m'] = errors[len(errors)//2] if errors else None
    return counts, sectors


def renderer():
    log = Path.home()/'.gz/rendering/ogre2.log'
    try:
        return next((line.split('=', 1)[1].strip() for line in log.read_text().splitlines() if 'GL_RENDERER' in line), None)
    except OSError:
        return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--scans', type=int, default=5, help='Scans compared per scanner and pose')
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    from ament_index_python.packages import get_package_share_directory
    sim_share = Path(get_package_share_directory('amr_simulation'))
    sys.path.insert(0, str(sim_share/'scripts'))
    from run_acceptance import simulation_processes, stop
    if simulation_processes():
        raise SystemExit('Finish the existing simulation first')
    os.environ.update(GZ_PARTITION=f'phase5_1_lidar_{os.getpid()}', ROS_DOMAIN_ID='93', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    from validate_runtime import Acceptance
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    items = []
    shapes(sim_share/'models', 'wh_building', [0.]*6, items)
    report = {'agree_threshold_m': AGREE_M, 'max_missing_fraction': MAX_MISSING_FRACTION,
              'environment': {'NOUVEAU_USE_ZINK': os.environ.get('NOUVEAU_USE_ZINK', 'unset (sim.launch.py default 0)')},
              'unmodelled_visuals': sum(i[0] == 'unmodelled' for i in items), 'poses': {}, 'passed': False}
    log = (out/'simulation.log').open('w')
    sim = subprocess.Popen(['ros2', 'launch', 'amr_simulation', 'warehouse.launch.py', 'scenario:=normal',
                            'mapping_only:=true', 'gui:=false'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    acc = None
    scans = {'/scan': [], '/top_scan': []}
    try:
        acc = Acceptance('normal', out)
        for topic in scans:
            acc.node.create_subscription(LaserScan, topic, lambda m, t=topic: scans[t].append(m), qos_profile_sensor_data)
        acc.until(lambda: 'warehouse_amr' in acc.poses and all(scans.values()), 180)
        mounts = {}
        for topic, msg in ((t, v[-1]) for t, v in scans.items()):
            acc.until(lambda f=msg.header.frame_id: acc.buffer.can_transform('base_link', f, rclpy.time.Time()), 30)
            tr = acc.buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time()).transform.translation
            mounts[topic] = (tr.x, tr.y, tr.z)
        report['mounts_base_link'] = mounts
        report['gl_renderer'] = renderer()
        for name, target in POSES.items():
            acc.set_pose(target)
            settle = acc.now()+1.5
            acc.until(lambda: acc.now() >= settle, 60)
            for v in scans.values():
                v.clear()
            acc.until(lambda: all(len(v) >= a.scans for v in scans.values()), 120)
            r = acc.robot()
            q = r['orientation']
            yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
            x, y, z = (r['position'][k] for k in 'xyz')
            entry = {'robot_pose': [x, y, z, yaw]}
            for topic, msgs in scans.items():
                mx, my, mz = mounts[topic]
                origin = (x+math.cos(yaw)*mx-math.sin(yaw)*my, y+math.sin(yaw)*mx+math.cos(yaw)*my, z+mz)
                totals, sectors = {}, {}
                for msg in msgs[:a.scans]:
                    counts, sec = compare(msg, origin, yaw, items)
                    for k, v in counts.items():
                        if k != 'median_agree_error_m':
                            totals[k] = totals.get(k, 0)+v
                    totals.setdefault('median_agree_error_m', []).append(counts['median_agree_error_m'])
                    for s, b in sec.items():
                        acc_b = sectors.setdefault(s, {'beams': 0, 'missing': 0})
                        acc_b['beams'] += b['beams']
                        acc_b['missing'] += b['missing']
                beams = sum(v for k, v in totals.items() if k != 'median_agree_error_m')
                med = sorted(m for m in totals.pop('median_agree_error_m') if m is not None)
                totals.update(beams=beams, missing_fraction=totals['missing']/beams,
                              median_agree_error_m=med[len(med)//2] if med else None,
                              missing_by_sector_deg={s: round(b['missing']/b['beams'], 3) for s, b in sorted(sectors.items())})
                entry[topic] = totals
            report['poses'][name] = entry
            print(name, json.dumps({t: {k: entry[t][k] for k in ('missing_fraction', 'mismatch', 'median_agree_error_m')} for t in scans}), flush=True)
        report['passed'] = all(e[t]['missing_fraction'] <= MAX_MISSING_FRACTION for e in report['poses'].values() for t in scans)
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        if acc is not None:
            acc.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        stop(sim)
        log.close()
        deadline = time.monotonic()+15
        while simulation_processes() and time.monotonic() < deadline:
            time.sleep(.2)
        report['remaining_processes'] = simulation_processes()
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'poses'}, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
