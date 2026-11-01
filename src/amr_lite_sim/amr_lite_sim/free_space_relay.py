"""Mapping-only relay: no-return beams become just-below-maximum readings for SLAM Toolbox.

Karto ignores readings at or beyond the sensor maximum (including +inf), so a beam that sees
open floor beyond range clears nothing and large open areas stay "unknown". This relay copies
/top_scan to /top_scan_mapping with +inf replaced by range_max - 0.01 m; SLAM then runs with
max_laser_range below that value, so those beams clear free space up to max_laser_range and
never mark an endpoint. Navigation, AMCL and costmaps keep using the unmodified scans.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class FreeSpaceRelay(Node):
    def __init__(self):
        super().__init__('free_space_relay')
        source = self.declare_parameter('input', '/top_scan').value
        target = self.declare_parameter('output', '/top_scan_mapping').value
        self.margin = self.declare_parameter('margin', 0.01).value
        self.pub = self.create_publisher(LaserScan, target, qos_profile_sensor_data)
        self.create_subscription(LaserScan, source, self.relay, qos_profile_sensor_data)

    def relay(self, msg):
        limit = msg.range_max-self.margin
        msg.ranges = [limit if r == math.inf else r for r in msg.ranges]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FreeSpaceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
