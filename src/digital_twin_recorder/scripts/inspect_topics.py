#!/usr/bin/env python3
"""Inventory of the live ROS graph for the recorder: type, publishers' QoS and measured rate.

Waits for discovery, subscribes to every requested topic with a QoS compatible with its
publishers, counts messages for ``--duration`` wall seconds and prints one JSON report.
Run with a sourced workspace (any interpreter that has rclpy); it never publishes.
"""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

DEFAULT = ['/odom', '/imu/data', '/lite_sim/world_state', '/lite_sim/events', '/lite_sim/obstacles',
           '/navigate_to_pose/_action/status', '/navigation/diagnostics', '/plan', '/tf', '/tf_static',
           '/scan', '/top_scan', '/clock']


def name(policy):
    return policy.name if hasattr(policy, 'name') else str(policy)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('topics', nargs='*', default=DEFAULT)
    p.add_argument('--duration', type=float, default=10.)
    p.add_argument('--discovery', type=float, default=8.)
    p.add_argument('--output')
    a = p.parse_args()
    rclpy.init()
    node = Node('recorder_topic_inspector')
    deadline = time.monotonic()+a.discovery
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    report, counts, sizes = {}, {}, {}
    for topic in a.topics:
        types = dict(node.get_topic_names_and_types()).get(topic) or []
        publishers = node.get_publishers_info_by_topic(topic)
        entry = report[topic] = {'types': types, 'publishers': [
            {'node': f'{i.node_namespace.rstrip("/")}/{i.node_name}', 'reliability': name(i.qos_profile.reliability),
             'durability': name(i.qos_profile.durability)} for i in publishers]}
        if len(types) != 1 or not publishers:
            continue
        reliable = all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in publishers)
        latched = all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in publishers)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
        entry['subscription_qos'] = {'reliability': name(qos.reliability), 'durability': name(qos.durability)}
        counts[topic] = 0
        sizes[topic] = 0

        def received(msg, topic=topic):
            counts[topic] += 1
            sizes[topic] += len(msg)
        node.create_subscription(get_message(types[0]), topic, received, qos, raw=True)
    start = time.monotonic()
    while time.monotonic()-start < a.duration:
        rclpy.spin_once(node, timeout_sec=.05)
    elapsed = time.monotonic()-start
    for topic, count in counts.items():
        report[topic].update(messages=count, rate_hz=round(count/elapsed, 2),
                             mean_serialized_bytes=round(sizes[topic]/count) if count else None)
    node.destroy_node()
    rclpy.shutdown()
    text = json.dumps({'duration_s': round(elapsed, 2), 'topics': report}, indent=1)
    if a.output:
        open(a.output, 'w').write(text+'\n')
    print(text)


if __name__ == '__main__':
    main()
