#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from rob_and_ros_pkg.robstride_controller import PositionController


class MotorSubscriber(Node):

    def __init__(self, motor_id):
        super().__init__('motor_sub_node')

        self.controller = PositionController(motor_id)

        if not self.controller.connect():
            self.get_logger().error("Motor connection failed")
            return

        self.subscription = self.create_subscription(
            Float64,
            '/motor_angle',
            self.angle_callback,
            10
        )

        self.get_logger().info("✅ Motor Subscriber Ready")

    def angle_callback(self, msg: Float64):

        angle = float(msg.data)

        # send relative command
        self.controller.set_angle_relative(angle)

        self.get_logger().info(
            f"Command: {angle:.2f}° | "
            f"Current: {math.degrees(self.controller.position):.2f}°"
        )

    def destroy_node(self):
        self.controller.stop_and_exit()
        super().destroy_node()


def main(args=None):

    if len(sys.argv) < 2:
        print("Usage: ros2 run rob_and_ros_pkg motor_sub_node <motor_id>")
        return

    motor_id = int(sys.argv[1])

    rclpy.init(args=args)
    node = MotorSubscriber(motor_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()