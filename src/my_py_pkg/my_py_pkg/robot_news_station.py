#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStation(Node):
    def __init__(self):
        super().__init__('robot_news_station')
        self.publisher_ = self.create_publisher(String, 'robot_news', 10)
        self.timer = self.create_timer(0.5, self.publish_news)
        self.get_logger().info('Robot News Station has been started.')

    def publish_news(self):
        msg = String()
        msg.data = 'Breaking news: ROS 2 is awesome!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published news: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()