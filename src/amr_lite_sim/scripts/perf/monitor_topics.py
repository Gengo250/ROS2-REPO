#!/usr/bin/env python3
"""Read-only ROS topic monitor: wall-clock arrival rates, gaps, /clock RTF and stalls.

Large messages are subscribed raw (no deserialization). Writes one JSON report.
"""
import argparse
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

RAW = {
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/camera/image_raw': 'sensor_msgs/msg/Image',
    '/camera/depth/image_raw': 'sensor_msgs/msg/Image',
    '/camera/points': 'sensor_msgs/msg/PointCloud2',
    '/warehouse/camera_01/image_raw': 'sensor_msgs/msg/Image',
    '/digital_twin/simulation/poses': 'tf2_msgs/msg/TFMessage',
    '/local_costmap/costmap': 'nav_msgs/msg/OccupancyGrid',
    '/cmd_vel': 'geometry_msgs/msg/TwistStamped',
}


def stats(values):
    if not values:
        return None
    v = sorted(values)
    pick = lambda q: v[min(len(v)-1, int(q*(len(v)-1)))]
    return {'n': len(v), 'mean': sum(v)/len(v), 'p50': pick(.5), 'p95': pick(.95), 'p99': pick(.99), 'max': v[-1]}


class Monitor(Node):
    def __init__(self, raw_topics):
        super().__init__('perf_monitor', parameter_overrides=[])
        self.arrivals = {}
        self.bytes = {}
        self.clock = []  # (wall, sim)
        self.odom = []   # (wall, sim_stamp, x, y, vx, wz)
        self.scan_age = {'/scan': [], '/top_scan': []}
        best = qos_profile_sensor_data
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Clock, '/clock', self.on_clock, QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, '/odom', self.on_odom, best)
        for t in ('/scan', '/top_scan'):
            self.create_subscription(LaserScan, t, lambda m, t=t: self.on_scan(t, m), best)
        self.create_subscription(TFMessage, '/tf', lambda m: self.mark('/tf'), rel)
        from rosidl_runtime_py.utilities import get_message
        for topic in raw_topics:
            kind = get_message(RAW[topic])
            qos = rel if topic in ('/local_costmap/costmap', '/cmd_vel') else best
            self.create_subscription(kind, topic, lambda m, t=topic: self.mark(t, len(m)), qos, raw=True)

    def mark(self, topic, size=0):
        self.arrivals.setdefault(topic, []).append(time.monotonic())
        self.bytes[topic] = self.bytes.get(topic, 0)+size

    def on_clock(self, m):
        self.clock.append((time.monotonic(), m.clock.sec+m.clock.nanosec*1e-9))

    def on_odom(self, m):
        self.mark('/odom')
        s = m.header.stamp
        self.odom.append((time.monotonic(), s.sec+s.nanosec*1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                          m.twist.twist.linear.x, m.twist.twist.angular.z))

    def on_scan(self, topic, m):
        self.mark(topic)
        if self.clock:
            s = m.header.stamp
            self.scan_age[topic].append(self.clock[-1][1]-(s.sec+s.nanosec*1e-9))

    def report(self, duration):
        out = {'duration_wall_s': duration, 'topics': {}}
        for topic, times in sorted(self.arrivals.items()):
            gaps = [b-a for a, b in zip(times, times[1:])]
            out['topics'][topic] = {'hz': (len(times)-1)/(times[-1]-times[0]) if len(times) > 1 else 0,
                                    'messages': len(times), 'mbytes_per_s': self.bytes.get(topic, 0)/duration/1e6,
                                    'gap_s': stats(gaps)}
        c = self.clock
        if len(c) > 2:
            rtf_total = (c[-1][1]-c[0][1])/(c[-1][0]-c[0][0])
            windows = []
            i = 0
            for j in range(len(c)):
                while c[j][0]-c[i][0] > 1.0:
                    i += 1
                if c[j][0]-c[i][0] >= .95:
                    windows.append((c[j][1]-c[i][1])/(c[j][0]-c[i][0]))
            gaps = [b[0]-a[0] for a, b in zip(c, c[1:])]
            out['clock'] = {'rtf_mean': rtf_total, 'rtf_1s_windows': stats(windows),
                            'wall_gap_s': stats(gaps), 'stalls_over_100ms': sum(g > .1 for g in gaps),
                            'stalls_over_250ms': sum(g > .25 for g in gaps), 'stalls_over_500ms': sum(g > .5 for g in gaps),
                            'sim_seconds': c[-1][1]-c[0][1]}
        o = self.odom
        if len(o) > 2:
            dist = sum(math.hypot(b[2]-a[2], b[3]-a[3]) for a, b in zip(o, o[1:]))
            wall = o[-1][0]-o[0][0]
            sim = o[-1][1]-o[0][1]
            moving = [r for r in o if abs(r[4]) > .02]
            out['odom_motion'] = {'distance_m': dist, 'wall_s': wall, 'sim_s': sim,
                                  'speed_wall_m_s': dist/wall if wall > 0 else 0, 'speed_sim_m_s': dist/sim if sim > 0 else 0,
                                  'max_cmd_speed_reported_m_s': max(abs(r[4]) for r in o),
                                  'moving_fraction': len(moving)/len(o)}
        out['scan_age_sim_s'] = {t: stats(v) for t, v in self.scan_age.items()}
        return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=30)
    p.add_argument('--output', required=True)
    p.add_argument('--raw', nargs='*', default=list(RAW))
    a = p.parse_args()
    rclpy.init()
    node = Monitor(a.raw)
    start = time.monotonic()
    while time.monotonic()-start < a.duration:
        rclpy.spin_once(node, timeout_sec=.05)
    report = node.report(time.monotonic()-start)
    with open(a.output, 'w') as f:
        json.dump(report, f, indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
