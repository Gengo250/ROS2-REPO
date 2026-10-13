#! /usr/bin/env python3

from example_interfaces.msg import String
import rclpy
from rclpy.node import Node


class Smartphone(Node):

    def __init__(self):
        super().__init__('smartphone')

        self.subscription = self.create_subscription(
            String,
            'robot_news',
            self.callback_robot_news,
            10
        )

        self.get_logger().info('Smartphone node has been started.')

    def callback_robot_news(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = Smartphone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
