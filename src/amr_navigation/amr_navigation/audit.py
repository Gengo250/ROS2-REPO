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


def blind_arc(hits, wraps):
    """Longest run of beams that never returned; ``wraps`` joins the ends of a 360 deg scan."""
    values = hits+hits if wraps else hits
    best = run = 0
    for hit in values:
        run = 0 if hit else run+1
        best = max(best, run)
    return min(best, len(hits))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=15.)
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = Node('navigation_sensor_audit', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    samples, latest, seen = {}, {}, {}
    def receive(topic, msg):
        stamp = msg.clock if topic == '/clock' else msg.header.stamp
        samples.setdefault(topic, []).append((time.monotonic(), stamp.sec+stamp.nanosec/1e9))
        latest[topic] = msg
        if isinstance(msg, LaserScan):
            # Beams that returned at least once during the audit window.
            hits = seen.setdefault(topic, [False]*len(msg.ranges))
            for i, r in enumerate(msg.ranges[:len(hits)]):
                hits[i] = hits[i] or math.isfinite(r)
    subscriptions = [node.create_subscription(kind, topic, lambda m,t=topic: receive(t,m), qos_profile_sensor_data)
        for topic,kind in [('/scan',LaserScan),('/top_scan',LaserScan),('/odom',Odometry),('/imu/data',Imu),('/joint_states',JointState),('/clock',Clock)]]
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
        top=latest['/top_scan']
        report['top_scan'] = {k:getattr(top,k) for k in ['angle_min','angle_max','angle_increment','range_min','range_max']}
        report['top_scan'].update(frame=top.header.frame_id,samples=len(top.ranges),finite_ranges=sum(math.isfinite(r) for r in top.ranges))
        report['odom_to_top_laser'] = message_to_ordereddict(buffer.lookup_transform('odom',top.header.frame_id,rclpy.time.Time()))
        for key, topic, msg, wraps in (('scan','/scan',scan,False), ('top_scan','/top_scan',top,True)):
            report[key]['widest_blind_arc_deg'] = math.degrees(blind_arc(seen[topic], wraps)*msg.angle_increment)
        assert set(latest)=={'/scan','/top_scan','/odom','/imu/data','/joint_states','/clock'}
        assert top.header.frame_id=='top_laser_frame' and len(top.ranges)==720
        assert abs(top.angle_max-top.angle_min+top.angle_increment-2*math.pi)<.01
        assert 8 <= report['topics']['/top_scan']['sim_hz'] <= 11 and report['top_scan']['finite_ranges']>100
        assert scan.header.frame_id=='laser_frame' and len(scan.ranges)==361
        assert abs(scan.angle_max-scan.angle_min-math.pi)<.01
        assert 13 <= report['topics']['/scan']['sim_hz'] <= 16
        assert report['scan']['finite_ranges']>20
        # A renderer/driver fault blanks whole sectors (zink: +/-45..90 deg). In the
        # enclosed warehouse the front scanner never legitimately misses a 30 deg arc;
        # the 360 deg scanner may look out of an open door, so it is reported only.
        assert report['scan']['widest_blind_arc_deg'] < 30
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
