#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class MotorPublisher(Node):

    def __init__(self):
        super().__init__('motor_pub_node')

        self.publisher = self.create_publisher(
            Float64,
            '/motor_angle',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_angle)

        self.angle = 0.0
        self.get_logger().info("📤 Motor Publisher Started")

    def publish_angle(self):

        msg = Float64()

        # Example: toggle between 0 and 30 degrees
        if self.angle == 0.0:
            self.angle = 30.0
        else:
            self.angle = 0.0

        msg.data = self.angle

        self.publisher.publish(msg)

        self.get_logger().info(f"Published angle: {self.angle:.2f}°")


def main(args=None):

    rclpy.init(args=args)
    node = MotorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()