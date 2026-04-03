#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_sub_node')

        self.subscription = self.create_subscription(
            Float64,
            '/test_angle',
            self.callback,
            10
        )

        self.get_logger().info("✅ Subscriber Ready")

    def callback(self, msg: Float64):
        value = float(msg.data)
        self.get_logger().info(f"Received: {value:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()