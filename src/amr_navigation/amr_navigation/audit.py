"""Bounded, read-only sensor/TF readiness audit against the real ROS graph."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=15.)
    args = parser.parse_args()
    rclpy.init()
    node = Node('navigation_sensor_audit', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    samples, latest = {}, {}
    def receive(topic, msg):
        stamp = msg.clock if topic == '/clock' else msg.header.stamp
        samples.setdefault(topic, []).append((time.monotonic(), stamp.sec+stamp.nanosec/1e9))
        latest[topic] = msg
    subscriptions = [node.create_subscription(kind, topic, lambda m,t=topic: receive(t,m), qos_profile_sensor_data)
        for topic,kind in [('/scan',LaserScan),('/odom',Odometry),('/imu/data',Imu),('/joint_states',JointState),('/clock',Clock)]]
    start = time.monotonic()
    while time.monotonic()-start < args.duration:
        rclpy.spin_once(node, timeout_sec=.1)
    report = {'duration_wall_s':time.monotonic()-start, 'topics':{}, 'passed':False}
    try:
        for topic, values in samples.items():
            wall, sim = values[-1][0]-values[0][0], values[-1][1]-values[0][1]
            report['topics'][topic] = {'messages':len(values),'wall_hz':(len(values)-1)/wall if wall else 0,
                'sim_hz':(len(values)-1)/sim if sim else 0,'last_stamp':values[-1][1],
                'monotonic_stamps':all(b[1]>a[1] for a,b in zip(values,values[1:]))}
        scan=latest['/scan']
        report['scan'] = {k:getattr(scan,k) for k in ['angle_min','angle_max','angle_increment','range_min','range_max','scan_time']}
        report['scan'].update(frame=scan.header.frame_id,samples=len(scan.ranges),finite_ranges=sum(math.isfinite(r) for r in scan.ranges))
        report['odom_to_laser'] = message_to_ordereddict(buffer.lookup_transform('odom',scan.header.frame_id,rclpy.time.Time()))
        report['tf_frames_yaml'] = buffer.all_frames_as_yaml()
        report['cmd_vel_publishers'] = [x.node_name for x in node.get_publishers_info_by_topic('/cmd_vel')]
        report['cmd_vel_subscribers'] = [{'node':x.node_name,'type':x.topic_type} for x in node.get_subscriptions_info_by_topic('/cmd_vel')]
        report['rtf'] = (samples['/clock'][-1][1]-samples['/clock'][0][1])/(samples['/clock'][-1][0]-samples['/clock'][0][0])
        assert set(latest)=={'/scan','/odom','/imu/data','/joint_states','/clock'}
        assert scan.header.frame_id=='laser_frame' and len(scan.ranges)==361
        assert abs(scan.angle_max-scan.angle_min-math.pi)<.01
        assert 13 <= report['topics']['/scan']['sim_hz'] <= 16
        assert report['scan']['finite_ranges']>20
        assert all(v['monotonic_stamps'] for v in report['topics'].values())
        assert report['cmd_vel_subscribers']==[{'node':'diff_drive_controller','type':'geometry_msgs/msg/TwistStamped'}]
        report['passed']=True
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report,indent=2))

if __name__=='__main__': main()
