#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_pub_node')

        self.publisher = self.create_publisher(
            Float64,
            '/test_angle',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_value)

        self.value = 0.0
        self.get_logger().info("📤 Publisher Started")

    def publish_value(self):
        msg = Float64()

        # Example: increase value every second
        self.value += 10.0
        if self.value > 100.0:
            self.value = 0.0

        msg.data = self.value

        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {self.value:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()