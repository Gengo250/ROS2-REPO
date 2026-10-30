#!/usr/bin/env python3
"""Real-sensor coverage acceptance: does each scanner observe a walking worker from
the front, left, right and rear, and static structure all around the AMR?

Owns one person_crossing simulation (worker yielding enabled, no Nav2). The AMR is
placed beside human_01's crossing line with the existing acceptance helper; the
worker walks past it. Detections come only from real LaserScan messages: an endpoint
counts when it lies within DETECTION_M of the worker. Gazebo poses serve solely to
project those endpoints and to judge which physical object they hit (evaluation).
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

DETECTION_M = 0.45
# Worker line x = -8.7, y in [-2.5, 2.5]. Robot centre 1.5-1.7 m away, outside the
# worker's yield envelope, so the worker keeps walking past.
CASES = {'front': [-10.4, 0.0, 0.015, 0, 0, 0.0], 'rear': [-7.0, 0.0, 0.015, 0, 0, 0.0],
         'left': [-7.2, 0.0, 0.015, 0, 0, math.pi/2], 'right': [-7.2, 0.0, 0.015, 0, 0, -math.pi/2]}
STATIC = [-4.5, 0.0, 0.015, 0, 0, 0.0]   # aisle_01 / main junction: racks in all four quadrants


def quaternion_yaw(q):
    return math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--observe', type=float, default=14., help='Simulation seconds per placement')
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    from ament_index_python.packages import get_package_share_directory
    sim_scripts = Path(get_package_share_directory('amr_simulation'))/'scripts'
    sys.path.insert(0, str(sim_scripts))
    # Appended, not prepended: amr_description also has a validate_runtime.py.
    sys.path.append(str(Path(get_package_share_directory('amr_description'))/'scripts'))
    from amr_parameters import load_parameters
    from run_acceptance import simulation_processes, stop
    if simulation_processes():
        raise SystemExit('Finish the existing simulation first')
    os.environ.update(GZ_PARTITION=f'phase5_1_sensors_{os.getpid()}', ROS_DOMAIN_ID='94',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    from validate_runtime import Acceptance
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    params = load_parameters(Path(get_package_share_directory('amr_description')))
    mounts = {'/scan': (params['lidar_x'], 0.), '/top_scan': (params['top_lidar_x'], 0.)}
    report = {'detection_threshold_m': DETECTION_M, 'cases': {}, 'passed': False}
    log = (out/'simulation.log').open('w')
    sim = subprocess.Popen(['ros2', 'launch', 'amr_simulation', 'warehouse.launch.py', 'scenario:=person_crossing', 'gui:=false'],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    acc = None
    scans = {'/scan': [], '/top_scan': []}
    try:
        acc = Acceptance('person_crossing', out)
        for topic in scans:
            acc.node.create_subscription(LaserScan, topic, lambda m, t=topic: scans[t].append(m), qos_profile_sensor_data)
        acc.until(lambda: 'warehouse_amr' in acc.poses and 'human_01' in acc.poses and all(scans.values()), 180)

        def endpoints(topic, msg, robot):
            x, y, yaw = robot
            mx, my = mounts[topic]
            sx, sy = x+math.cos(yaw)*mx-math.sin(yaw)*my, y+math.sin(yaw)*mx+math.cos(yaw)*my
            return [(sx+r*math.cos(yaw+msg.angle_min+i*msg.angle_increment), sy+r*math.sin(yaw+msg.angle_min+i*msg.angle_increment))
                    for i, r in enumerate(msg.ranges) if math.isfinite(r) and msg.range_min <= r <= msg.range_max]

        def observe(seconds):
            for v in scans.values():
                v.clear()
            rows, end = [], acc.now()+seconds
            while acc.now() < end:
                rclpy.spin_once(acc.node, timeout_sec=.02)
                for topic, buffer in scans.items():
                    while buffer:
                        msg = buffer.pop(0)
                        r, h = acc.robot(), acc.poses['human_01']
                        robot = (r['position']['x'], r['position']['y'], quaternion_yaw(r['orientation']))
                        worker = (h['position']['x'], h['position']['y'])
                        pts = endpoints(topic, msg, robot)
                        dx, dy = worker[0]-robot[0], worker[1]-robot[1]
                        bearing = math.degrees(math.atan2(dy, dx)-robot[2])
                        rows.append({'topic': topic, 'sim': msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                     'worker_distance_m': math.hypot(dx, dy), 'worker_bearing_deg': (bearing+180) % 360-180,
                                     'nearest_endpoint_to_worker_m': min((math.hypot(px-worker[0], py-worker[1]) for px, py in pts), default=None),
                                     'finite_returns': len(pts), 'points': pts if len(rows) < 4 else None})
            return rows

        for name, pose in CASES.items():
            acc.set_pose(pose)
            rows = observe(a.observe)
            near = [r for r in rows if r['worker_distance_m'] <= 3.0]
            summary = {'robot_pose': pose, 'samples_worker_within_3m': {}}
            for topic in scans:
                mine = [r for r in near if r['topic'] == topic]
                hits = [r for r in mine if r['nearest_endpoint_to_worker_m'] is not None and r['nearest_endpoint_to_worker_m'] < DETECTION_M]
                summary['samples_worker_within_3m'][topic] = {
                    'scans': len(mine), 'detections': len(hits), 'rate': len(hits)/len(mine) if mine else None,
                    'closest_detected_m': min((r['worker_distance_m'] for r in hits), default=None),
                    'bearings_deg': [round(r['worker_bearing_deg']) for r in mine][::5]}
            summary['min_worker_distance_m'] = min((r['worker_distance_m'] for r in rows), default=None)
            report['cases'][name] = summary
            (out/f'{name}.json').write_text(json.dumps(rows)+'\n')
            print(name, json.dumps(summary['samples_worker_within_3m']), flush=True)
        acc.set_pose(STATIC)
        rows = observe(3.)
        top = [r for r in rows if r['topic'] == '/top_scan']
        quadrants = {}
        for r in [next(r for r in top if r['points'])]:  # point lists are kept for the first rows only
            x, y = STATIC[0], STATIC[1]
            for px, py in r['points'] or []:
                q = int(((math.degrees(math.atan2(py-y, px-x))+360) % 360)//90)
                quadrants[q] = quadrants.get(q, 0)+1
        report['static_top_scan_returns_by_quadrant'] = quadrants
        (out/'static.json').write_text(json.dumps(rows)+'\n')
        report['contacts'] = dict(acc.contacts)
        top_ok = all(report['cases'][c]['samples_worker_within_3m']['/top_scan']['detections'] > 0 for c in CASES)
        report['passed'] = top_ok and len(quadrants) == 4
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
    plot(out)
    print(json.dumps({k: v for k, v in report.items() if k != 'cases'}, indent=2))
    if not report['passed']:
        raise SystemExit(1)


def plot(out):
    """Coverage diagram from the first recorded real scans of each placement."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 4, figsize=(20, 5))
    for ax, name in zip(axes, CASES):
        path = out/f'{name}.json'
        if not path.exists():
            continue
        rows = json.loads(path.read_text())
        for topic, colour in (('/scan', 'tab:orange'), ('/top_scan', 'tab:blue')):
            sample = next((r for r in rows if r['topic'] == topic and r['points']), None)
            if sample:
                xs, ys = zip(*sample['points'])
                ax.scatter(xs, ys, s=2, c=colour, label=topic)
        pose = CASES[name]
        ax.plot(pose[0], pose[1], 'k^', ms=10)
        ax.arrow(pose[0], pose[1], .8*math.cos(pose[5]), .8*math.sin(pose[5]), width=.05, color='k')
        ax.axvline(-8.7, color='tab:green', ls='--', label='worker line')
        ax.set(title=f'worker passes {name}', xlim=(pose[0]-6, pose[0]+6), ylim=(-6, 6), aspect='equal')
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out/'sensor-coverage.png', dpi=110)


if __name__ == '__main__':
    main()
