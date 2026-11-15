#!/usr/bin/env python3
"""Check the multi-robot lite twin's ROS interfaces against the fleet convention (fleet.py).

For every robot: each sensor topic has exactly one publisher (the engine) and carries that robot's
frame; its cmd_vel has exactly the engine as subscriber; its TF chain map → odom → base_link →
sensors exists with frames that belong to no other robot; and, if AMCL runs, its map → base_link
estimate converted to the world frame matches the engine's ground truth.
Usage: validate_fleet.py --output report.json [--robots amr_01,amr_02,amr_03] [--seconds 6]
"""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from amr_lite_sim import fleet

SENSORS = {'odom': (Odometry, 'odom'), 'scan': (LaserScan, 'laser_frame'), 'top_scan': (LaserScan, 'top_laser_frame'),
           'imu/data': (Imu, 'imu_link')}


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    p.add_argument('--robots', default='amr_01,amr_02,amr_03')
    p.add_argument('--seconds', type=float, default=6.)
    p.add_argument('--registration', default='')
    a = p.parse_args()
    robots = fleet.parse_robots(a.robots)
    if not a.registration:
        from ament_index_python.packages import get_package_share_directory
        a.registration = str(Path(get_package_share_directory('amr_navigation'))/'maps/campus_registration.json')
    reg = json.loads(Path(a.registration).read_text())['transform']
    rclpy.init()
    node = rclpy.create_node('validate_fleet')
    buffer = Buffer(node=node)
    TransformListener(buffer, node)
    frames, counts, truth, tf_children, static_children = {}, {}, {}, {}, {}
    for robot in robots:
        for name, (kind, _) in SENSORS.items():
            topic = fleet.topic(robot, name)
            counts[topic] = 0
            def callback(msg, topic=topic):
                counts[topic] += 1
                frames.setdefault(topic, set()).add(msg.header.frame_id)
            node.create_subscription(kind, topic, callback, qos_profile_sensor_data)

    def tf(msg, store):
        for t in msg.transforms:
            store.setdefault(t.child_frame_id, set()).add(t.header.frame_id)
    node.create_subscription(TFMessage, '/tf', lambda m: tf(m, tf_children), 100)
    node.create_subscription(TFMessage, '/tf_static', lambda m: tf(m, static_children),
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(TFMessage, '/digital_twin/simulation/poses', lambda m: truth.update(
        {t.child_frame_id: (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)) for t in m.transforms}),
        qos_profile_sensor_data)
    end = time.monotonic()+a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    report = {'robots': {}, 'checks': {}}
    all_frames = {}
    c, s = math.cos(reg['yaw']), math.sin(reg['yaw'])
    for robot in robots:
        prefix = fleet.frame_prefix(robot)
        entry = {'namespace': fleet.namespace(robot) or '/', 'frame_prefix': prefix, 'topics': {}}
        for name, (_, frame) in SENSORS.items():
            topic = fleet.topic(robot, name)
            publishers = [i.node_name for i in node.get_publishers_info_by_topic(topic)]
            entry['topics'][topic] = {'messages': counts[topic], 'rate_hz': round(counts[topic]/a.seconds, 1),
                                      'frames': sorted(frames.get(topic, ())), 'publishers': publishers,
                                      'ok': publishers == ['lite_sim'] and frames.get(topic) == {prefix+frame} and counts[topic] > 0}
        cmd = fleet.topic(robot, 'cmd_vel')
        subscribers = [i.node_name for i in node.get_subscriptions_info_by_topic(cmd)]
        command_publishers = [i.node_name for i in node.get_publishers_info_by_topic(cmd)]
        entry['cmd_vel'] = {'topic': cmd, 'subscribers': subscribers, 'publishers': command_publishers,
                            'ok': subscribers == ['lite_sim'] and len(command_publishers) <= 1}
        chain = [('map', prefix+'odom'), (prefix+'odom', prefix+'base_link'), (prefix+'base_link', prefix+'laser_frame'),
                 (prefix+'base_link', prefix+'top_laser_frame'), (prefix+'base_link', prefix+'camera_optical_frame')]
        entry['tf_chain'] = {f'{a_} -> {b_}': buffer.can_transform(a_, b_, rclpy.time.Time()) for a_, b_ in chain}
        try:
            t = buffer.lookup_transform('map', prefix+'base_link', rclpy.time.Time())
            mx, my, myaw = t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
            dx, dy = mx-reg['x'], my-reg['y']
            wx, wy = c*dx+s*dy, -s*dx+c*dy
            wyaw = math.atan2(math.sin(myaw-reg['yaw']), math.cos(myaw-reg['yaw']))
            gx, gy, gyaw = truth[robot]
            entry['localization'] = {'amcl_world': [round(wx, 3), round(wy, 3), round(wyaw, 4)], 'ground_truth': [round(gx, 3), round(gy, 3), round(gyaw, 4)],
                                     'error_m': round(math.hypot(wx-gx, wy-gy), 3),
                                     'error_rad': round(abs(math.atan2(math.sin(wyaw-gyaw), math.cos(wyaw-gyaw))), 4)}
        except Exception as error:   # no AMCL for this robot, or no ground truth
            entry['localization'] = {'unavailable': str(error)[:200]}
        robot_frames = {prefix+f for f in ('odom', 'base_link', 'laser_frame', 'top_laser_frame', 'imu_link', 'camera_optical_frame')}
        for f in robot_frames:
            all_frames.setdefault(f, []).append(robot)
        report['robots'][robot] = entry
    report['checks']['sensor_topics_independent'] = all(t['ok'] for r in report['robots'].values() for t in r['topics'].values())
    report['checks']['cmd_vel_one_engine_subscriber_each'] = all(r['cmd_vel']['ok'] for r in report['robots'].values())
    report['checks']['tf_chains_complete'] = all(all(r['tf_chain'].values()) for r in report['robots'].values())
    report['checks']['frames_unique_per_robot'] = all(len(v) == 1 for v in all_frames.values())
    # Every odom → base_link edge in /tf is published by exactly one parent per child frame.
    report['checks']['tf_single_parent'] = all(len(parents) == 1 for parents in {**tf_children, **static_children}.values())
    report['tf_children'] = {k: sorted(v) for k, v in sorted(tf_children.items())}
    errors = [r['localization'].get('error_m') for r in report['robots'].values()]
    report['checks']['localization_within_0_25_m'] = all(e is not None and e < .25 for e in errors)
    report['passed'] = all(report['checks'].values())
    Path(a.output).write_text(json.dumps(report, indent=1))
    print(json.dumps(report['checks'], indent=1))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
