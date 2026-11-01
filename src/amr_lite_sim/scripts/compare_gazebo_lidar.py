#!/usr/bin/env python3
"""Cross-check the lite engine against the optional gazebo_validation world.

Requires `ros2 launch amr_simulation gazebo_validation.launch.py` in the same ROS domain and
GZ_PARTITION. For each test pose the Gazebo AMR is placed with set_pose, one /scan and one
/top_scan are captured from gpu_lidar, and the lite LiDAR is computed at the same pose from the
same scene. Then the same velocity profile is applied to both drives and the motion compared.
Gazebo is only the reference here; nothing is fed back to the lite engine.
Usage: compare_gazebo_lidar.py --output report.json
"""
import argparse
import json
import math
import subprocess
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from atlas_scene.scene import Scene
from amr_lite_sim import robot_model
from amr_lite_sim.world import World

POSES = {'spawn_hall': (2., 0., math.pi/2), 'north_cross_aisle': (-30., 22.5, math.pi), 'rack_aisle': (-55., 36., math.pi/2),
         'production_aisle': (-12., 33., 0.), 'dock_aisle': (8., -45.5, 0.), 'charger_aisle': (63.4, 33., math.pi/2)}
AGREE_M = 0.05


def set_pose(x, y, yaw):
    req = (f'name: "warehouse_amr" position {{x: {x} y: {y} z: 0.015}} '
           f'orientation {{z: {math.sin(yaw/2)} w: {math.cos(yaw/2)}}}')
    out = subprocess.run(['gz', 'service', '-s', '/world/campus_validation/set_pose', '--reqtype', 'gz.msgs.Pose',
                          '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', req], capture_output=True, text=True, timeout=15)
    if 'data: true' not in out.stdout:
        raise RuntimeError('set_pose refused: '+out.stdout+out.stderr)


def compare(measured, expected):
    m, e = np.asarray(measured, dtype=float), np.asarray(expected, dtype=float)
    both = np.isfinite(m) & np.isfinite(e)
    diff = np.abs(m[both]-e[both])
    return {'beams': int(len(m)), 'both_finite': int(both.sum()), 'agree_within_5cm': float((diff <= AGREE_M).mean()) if both.any() else None,
            'median_abs_diff_m': float(np.median(diff)) if both.any() else None, 'p95_abs_diff_m': float(np.percentile(diff, 95)) if both.any() else None,
            'gazebo_only_returns': int((np.isfinite(m) & ~np.isfinite(e)).sum()), 'lite_only_returns': int((~np.isfinite(m) & np.isfinite(e)).sum())}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    a = p.parse_args()
    scene, model = Scene.load(), robot_model.load()
    world = World(scene, model, 'empty')
    rclpy.init()
    node = rclpy.create_node('lite_gazebo_comparison')
    latest = {}
    for topic in ('/scan', '/top_scan'):
        node.create_subscription(LaserScan, topic, lambda m, t=topic: latest.__setitem__(t, (time.monotonic(), m)), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', lambda m: latest.__setitem__('/odom', (time.monotonic(), m)), qos_profile_sensor_data)
    cmd = node.create_publisher(TwistStamped, '/cmd_vel', 10)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def drive(v, w, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            msg = TwistStamped()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x, msg.twist.angular.z = v, w
            cmd.publish(msg)
            spin(.05)
    spin(3.)
    report = {'poses': {}, 'note': 'gazebo_validation world generated from the same scene; Gazebo gpu_lidar vs lite ray casting at identical poses'}
    for name, (x, y, yaw) in POSES.items():
        drive(0., 0., .5)
        set_pose(x, y, yaw)
        moved = time.monotonic()
        spin(2.5)
        robot = world.robots['amr_01']
        robot.drive.x, robot.drive.y, robot.drive.yaw = x, y, yaw
        entry = {}
        for topic, spec in (('/scan', model.front), ('/top_scan', model.top)):
            got = latest.get(topic)
            if got is None or got[0] < moved+1.:
                entry[topic] = {'error': 'no fresh Gazebo scan'}
                continue
            entry[topic] = compare(got[1].ranges, world.scan('amr_01', spec))
        report['poses'][name] = entry
        print(name, json.dumps({t: {k: v for k, v in e.items() if k in ('agree_within_5cm', 'median_abs_diff_m', 'gazebo_only_returns', 'lite_only_returns')} for t, e in entry.items()}))
    # Motion, in simulation time: step to 0.5 m/s, then an arc (0.3 m/s, 0.4 rad/s).
    odom = []
    node.create_subscription(Odometry, '/odom', lambda m: odom.append(
        (m.header.stamp.sec+m.header.stamp.nanosec*1e-9, m.twist.twist.linear.x, m.twist.twist.angular.z)), qos_profile_sensor_data)
    drive(0., 0., .5)
    set_pose(2., 0., math.pi/2)
    drive(0., 0., 2.)
    odom.clear()
    drive(.5, 0., 6.)
    arc_start = len(odom)
    drive(.3, .4, 5.)
    drive(0., 0., 2.)

    def metrics(samples, arc_index):
        on = next(t for t, v, _ in samples if v > .01)
        full = next(t for t, v, _ in samples if v >= .49)
        straight = [v for t, v, _ in samples[:arc_index] if t > full+.5]
        arc = [w for _, _, w in samples[arc_index:] if abs(w) > .05][-20:]
        return {'ramp_0_to_0p49_s': full-on, 'steady_v_m_s': float(np.median(straight)), 'steady_w_rad_s': float(np.median(arc))}
    lite = World(scene, model, 'empty')
    trace, t = [], 0.
    for v, w, seconds in ((.5, 0., 6.), (.3, .4, 5.)):
        for _ in range(int(seconds/.01)):
            lite.command('amr_01', v, w)
            lite.step(.01)
            t += .01
            trace.append((t, lite.robots['amr_01'].drive.v, lite.robots['amr_01'].drive.w))
    report['motion'] = {'gazebo': metrics(odom, arc_start), 'lite': metrics(trace, 600),
                        'odom_samples': len(odom), 'note': 'simulation-time metrics; Gazebo from diff_drive_controller /odom'}
    print('motion', json.dumps(report['motion']))
    json.dump(report, open(a.output, 'w'), indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
