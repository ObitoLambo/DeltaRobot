#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# Import your existing MIT controller class
from rob_and_ros_pkg.position_control import PositionControllerMIT


class MITMotorNode(Node):

    def __init__(self, motor_id: int):
        super().__init__('mit_motor_node')

        # Create MIT controller
        self.controller = PositionControllerMIT(motor_id)

        if not self.controller.connect():
            self.get_logger().error("Motor connection failed")
            return

        # ROS subscriber
        self.subscription = self.create_subscription(
            Float64,
            '/motor_angle',
            self.angle_callback,
            10
        )

        self.get_logger().info("🚀 MIT Motor Node Ready")

    def angle_callback(self, msg: Float64):
        angle = float(msg.data)

        # Directly use your existing function
        self.controller.set_angle(angle)

        self.get_logger().info(
            f"Command: {angle:.2f}°"
        )

    def destroy_node(self):
        self.controller.stop_and_exit()
        super().destroy_node()


def main(args=None):

    if len(sys.argv) < 2:
        print("Usage: ros2 run rob_and_ros_pkg mit_motor_node <motor_id>")
        return

    motor_id = int(sys.argv[1])

    rclpy.init(args=args)
    node = MITMotorNode(motor_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()