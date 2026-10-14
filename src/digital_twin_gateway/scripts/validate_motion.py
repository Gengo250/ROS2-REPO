#!/usr/bin/env python3
"""Controlled ROS-only integration probe; always sends stamped zero commands."""
import argparse
import json
from pathlib import Path
import time
import httpx
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('twin_motion_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    odometry = []
    def receive(msg):
        odometry[:] = [msg]
    sub = node.create_subscription(Odometry, '/odom', receive, qos_profile_sensor_data)
    pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    report = {}
    def drive(speed, duration):
        start, deadline = node.get_clock().now().nanoseconds, time.monotonic()+75
        while (node.get_clock().now().nanoseconds-start)/1e9 < duration:
            if time.monotonic() > deadline:
                raise TimeoutError('Simulation clock stopped during motion validation')
            message = TwistStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            message.twist.linear.x = speed
            pub.publish(message)
            rclpy.spin_once(node, timeout_sec=.03)
    try:
        deadline = time.monotonic()+30
        while not odometry or node.get_clock().now().nanoseconds == 0:
            if time.monotonic() > deadline:
                raise TimeoutError('No real odometry/clock received')
            rclpy.spin_once(node, timeout_sec=.1)
        before = httpx.get('http://127.0.0.1:8000/api/v1/state').json()
        report['before'] = before
        report['ros_before_x'] = odometry[0].pose.pose.position.x
        drive(.12, 2.0)
        drive(0.0, 1.0)
        time.sleep(.3)
        after = httpx.get('http://127.0.0.1:8000/api/v1/state').json()
        report['after'] = after
        report['ros_after_x'] = odometry[0].pose.pose.position.x
        displacement = after['robots'][0]['pose']['position']['x'] - before['robots'][0]['pose']['position']['x']
        assert .03 < displacement < .5, displacement
        assert abs(after['robots'][0]['velocity']['linear']['x']) < .01
        assert abs(report['ros_after_x'] - after['robots'][0]['pose']['position']['x']) < .02
        report['displacement_m'] = displacement
        report['stopped'] = True
    finally:
        # A finite wall-clock stop sequence still executes if /clock stalls.
        for _ in range(20):
            message = TwistStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            pub.publish(message)
            rclpy.spin_once(node, timeout_sec=.025)
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
