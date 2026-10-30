#!/usr/bin/env python3
"""Read-only SLAM drift recorder synchronized by SIMULATION time.

For every LaserScan stamp it records the SLAM estimate (map → base_link at that
stamp), wheel odometry (odom → base_link at that stamp) and Gazebo ground truth
interpolated to the same stamp. Ground truth is evaluation-only; nothing is
published. The active experiment segment is read from --label-file.
"""
import argparse
import bisect
import json
import math
import signal
import time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose(t):
    return [t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)]


def interpolate(samples, stamp, gap=.15):
    """Linear interpolation of [stamp, x, y, yaw] samples; None without a close bracket."""
    i = bisect.bisect_left([s[0] for s in samples], stamp)
    if i == 0 or i >= len(samples):
        return None
    a, b = samples[i-1], samples[i]
    if b[0]-a[0] > gap:
        return None
    k = (stamp-a[0])/(b[0]-a[0]) if b[0] > a[0] else 0.
    return [a[1]+k*(b[1]-a[1]), a[2]+k*(b[2]-a[2]), wrap(a[3]+k*wrap(b[3]-a[3]))]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--label-file', type=Path, help='Driver-written current segment label')
    p.add_argument('--scan', default='/scan')
    p.add_argument('--duration', type=float, default=3600.)
    a = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    n = rclpy.create_node('slam_drift_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=n, cache_time=rclpy.duration.Duration(seconds=30))
    TransformListener(buffer, n)
    truth, pending, rows, scans, commands = [], [], [], [], []
    running = [True]

    def ground_truth(msg):
        for t in msg.transforms:
            if t.header.frame_id == 'world' and t.child_frame_id == 'amr_01':
                stamp = t.header.stamp.sec+t.header.stamp.nanosec/1e9
                if not truth or stamp > truth[-1][0]:
                    truth.append([stamp, *pose(t)])

    def scan(msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        scans.append([stamp, time.monotonic(), sum(1 for r in msg.ranges if math.isfinite(r))])
        pending.append((stamp, msg.header.stamp, time.monotonic()))

    n.create_subscription(TFMessage, '/digital_twin/simulation/poses', ground_truth, qos_profile_sensor_data)
    n.create_subscription(LaserScan, a.scan, scan, qos_profile_sensor_data)
    n.create_subscription(TwistStamped, '/cmd_vel', lambda m: commands.append(
        [m.header.stamp.sec+m.header.stamp.nanosec/1e9, m.twist.linear.x, m.twist.angular.z]), 10)
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))
    signal.signal(signal.SIGTERM, lambda *_: running.__setitem__(0, False))
    end = time.monotonic()+a.duration
    label = ''
    while running[0] and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.02)
        if a.label_file and a.label_file.exists():
            label = a.label_file.read_text().strip()
        keep = []
        for stamp, ros_stamp, received in pending:
            try:
                slam = pose(buffer.lookup_transform('map', 'base_link', Time.from_msg(ros_stamp)))
                odom = pose(buffer.lookup_transform('odom', 'base_link', Time.from_msg(ros_stamp)))
            except Exception:
                if time.monotonic()-received < 1.5:
                    keep.append((stamp, ros_stamp, received))
                continue
            gt = interpolate(truth, stamp)
            if gt is not None:
                rows.append({'sim': stamp, 'label': label, 'slam': slam, 'odom': odom, 'truth': gt})
        pending[:] = keep
    sims = [s[0] for s in scans]
    report = {'scan_topic': a.scan, 'samples': rows, 'commands': commands,
              'scan': {'messages': len(scans),
                       'sim_rate_hz': (len(sims)-1)/(sims[-1]-sims[0]) if len(sims) > 1 and sims[-1] > sims[0] else None,
                       'rtf': (sims[-1]-sims[0])/(scans[-1][1]-scans[0][1]) if len(scans) > 1 and scans[-1][1] > scans[0][1] else None,
                       'median_finite_returns': sorted(s[2] for s in scans)[len(scans)//2] if scans else None},
              'note': 'Ground truth interpolated to each scan stamp; evaluation only.'}
    a.output.write_text(json.dumps(report)+'\n')
    print(json.dumps({'samples': len(rows), **report['scan']}))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
