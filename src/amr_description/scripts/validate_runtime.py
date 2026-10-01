#!/usr/bin/env python3
"""Integration acceptance checks against an already running sim.launch.py."""
import json
import math
import struct
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener
from amr_parameters import ROOT, load_parameters


def require(ok, message):
    if not ok:
        raise AssertionError(message)
    print('PASS:',message,flush=True)


def ground_truth():
    text = subprocess.check_output(['gz','topic','-e','-t',
        '/world/amr_validation/dynamic_pose/info','-n','1','--json-output'],text=True,timeout=15)
    message = json.loads(text)
    return next(pose for pose in message['pose'] if pose['name']=='warehouse_amr')


def yaw(quaternion):
    q = quaternion
    return math.atan2(2*(q.get('w',1)*q.get('z',0)+q.get('x',0)*q.get('y',0)),
                      1-2*(q.get('y',0)**2+q.get('z',0)**2))


def run():
    parameters = load_parameters()
    rclpy.init()
    node = rclpy.create_node('amr_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    received,counts,subscriptions = {},{},[]
    topics = {'/scan':LaserScan,'/imu/data':Imu,'/camera/image_raw':Image,
              '/camera/depth/image_raw':Image,'/camera/camera_info':CameraInfo,
              '/camera/points':PointCloud2,'/odom':Odometry,'/joint_states':JointState}
    def receive(topic,message):
        received[topic] = message
        counts[topic] = counts.get(topic,0)+1
    for topic,kind in topics.items():
        subscriptions.append(node.create_subscription(kind,topic,lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data))
    buffer = Buffer()
    listener = TransformListener(buffer,node)
    publisher = node.create_publisher(TwistStamped,'/cmd_vel',10)
    def spin_until(predicate,timeout=60):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline:
                raise TimeoutError(f'Runtime condition timed out; message counts: {counts}')
            rclpy.spin_once(node,timeout_sec=0.05)
    def sim_time():
        return node.get_clock().now().nanoseconds/1e9
    def wait_sim(seconds):
        start = sim_time()
        spin_until(lambda:sim_time()-start>=seconds,timeout=max(60,seconds*20))
    def drive(linear,angular,seconds):
        start = sim_time()
        deadline = time.monotonic()+seconds*30
        while sim_time()-start<seconds:
            require_time = time.monotonic()<deadline
            if not require_time:
                raise TimeoutError('Simulation stopped during motion')
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = linear
            msg.twist.angular.z = angular
            publisher.publish(msg)
            rclpy.spin_once(node,timeout_sec=0.02)
    try:
        spin_until(lambda:all(counts.get(t,0)>=3 for t in topics))
        wait_sim(2)
        for topic in topics:
            message = received[topic]
            age = sim_time()-message.header.stamp.sec-message.header.stamp.nanosec/1e9
            require(-0.05<age<1.0,topic+' contains fresh samples')
        require(received['/scan'].header.frame_id=='laser_frame','scan frame is laser_frame')
        require(received['/imu/data'].header.frame_id=='imu_link','IMU frame is imu_link')
        for topic in ('/camera/image_raw','/camera/depth/image_raw','/camera/camera_info'):
            require(received[topic].header.frame_id=='camera_optical_frame',topic+' optical frame')
        require(received['/camera/points'].header.frame_id=='camera_depth_frame','point cloud uses +X-forward lens frame')
        for frame in ('chassis_link','platform_link','left_wheel_link','right_wheel_link','front_caster_link',
                      'rear_caster_link','laser_frame','imu_link','camera_optical_frame','camera_depth_frame'):
            spin_until(lambda:buffer.can_transform('base_link',frame,rclpy.time.Time()),timeout=10)
        require(True,'all physical and sensor TFs resolve from base_link')
        scan = received['/scan']
        require(len(scan.ranges)==parameters['lidar_samples'],'scan sample count')
        centre_range = scan.ranges[len(scan.ranges)//2]
        require(abs(centre_range-(2.8-parameters['lidar_x']))<0.06,'LiDAR sees known obstacle at expected distance')
        require(min(scan.ranges)>1.0,'LiDAR has no near self returns in the test world')
        depth = received['/camera/depth/image_raw']
        require(depth.encoding=='32FC1','depth encoding is float metres')
        pixel = depth.height//2*depth.step+depth.width//2*4
        centre_depth, = struct.unpack_from('>f' if depth.is_bigendian else '<f',depth.data,pixel)
        expected_depth = 2.8-(parameters['camera_x']+parameters['camera_length']/2+parameters['camera_lens_gap'])
        require(abs(centre_depth-expected_depth)<0.06,'RGB-D depth sees the known obstacle')
        cloud = received['/camera/points']
        fields = {field.name:field.offset for field in cloud.fields}
        offset = cloud.height//2*cloud.row_step+cloud.width//2*cloud.point_step
        point = {axis:struct.unpack_from('>f' if cloud.is_bigendian else '<f',cloud.data,offset+fields[axis])[0] for axis in 'xyz'}
        require(abs(point['x']-centre_depth)<0.06 and abs(point['y'])<0.03 and abs(point['z'])<0.03,
                'PointCloud2 coordinates agree with +X-forward frame and depth')
        imu = received['/imu/data']
        require(abs(imu.linear_acceleration.z-9.81)<0.15,'IMU at rest measures gravity consistently')
        initial = ground_truth()
        require(abs(initial['position'].get('z',0))<0.003,'base settled on floor without sinking')
        require(abs(initial['orientation'].get('x',0))<0.01 and abs(initial['orientation'].get('y',0))<0.01,
                'initial roll and pitch stable')
        odom_before = received['/odom'].pose.pose.position.x
        drive(0.2,0.0,2.5)
        # Deliberately stop publishing to check the actual stale-command brake.
        wait_sim(2.0)
        forward = ground_truth()
        displacement = forward['position'].get('x',0)-initial['position'].get('x',0)
        odom_displacement = received['/odom'].pose.pose.position.x-odom_before
        require(0.30<displacement<0.75,'positive command moves physical robot along +X')
        require(abs(odom_displacement-displacement)<0.08,'encoder odometry agrees with physical translation')
        require(abs(received['/odom'].twist.twist.linear.x)<0.02,'command timeout brakes the base')
        require(max(abs(v) for v in received['/joint_states'].velocity)<0.1,'wheels stop after timeout')
        drive(0.0,0.4,2.5)
        wait_sim(2.0)
        turned = ground_truth()
        angle = yaw(turned['orientation'])-yaw(forward['orientation'])
        require(0.5<angle<1.5,'positive angular command turns counter-clockwise')
        require(abs(turned['position'].get('z',0))<0.003,'floor contact maintained after drive and turn')
        require(abs(turned['orientation'].get('x',0))<0.01 and abs(turned['orientation'].get('y',0))<0.01,
                'roll and pitch stable after motion')
        drive(0.0,0.0,0.5)
        report = {'status':'passed','config_sha256':parameters['config_sha256'],
                  'topic_samples':counts,'lidar_centre_m':centre_range,'depth_centre_m':centre_depth,
                  'pointcloud_centre_depth_frame_m':point,'initial_pose':initial,
                  'forward_pose':forward,'turned_pose':turned,'translation_m':displacement,
                  'odom_translation_m':odom_displacement,'turn_radians':angle,
                  'simulation_seconds_at_completion':sim_time()}
        (ROOT/'validation/runtime.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        return report
    finally:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
