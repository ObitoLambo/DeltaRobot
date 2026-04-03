#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class DeltaJointPublisher(Node):

    def __init__(self):
        super().__init__('delta_joint_publisher')

        self.pub_1 = self.create_publisher(Float64, '/delta/joint_angle_1', 10)
        self.pub_2 = self.create_publisher(Float64, '/delta/joint_angle_2', 10)
        self.pub_3 = self.create_publisher(Float64, '/delta/joint_angle_3', 10)

        self.timer_period = 0.1   # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.angle_deg = 0.0
        self.step_deg = 1.0       # increase 1 degree each cycle
        self.max_deg = 90.0

        self.get_logger().info('Delta Joint Publisher Started')
        self.get_logger().info('Publishing 0 -> 90 degree loop')
        self.get_logger().info('Output topics: /delta/joint_angle_1, _2, _3')
        self.get_logger().info('Published unit: radians')

    def timer_callback(self):
        angle_rad = math.radians(self.angle_deg)

        msg1 = Float64()
        msg2 = Float64()
        msg3 = Float64()

        # all 3 motors get the same command
        msg1.data = angle_rad
        msg2.data = angle_rad
        msg3.data = angle_rad

        self.pub_1.publish(msg1)
        self.pub_2.publish(msg2)
        self.pub_3.publish(msg3)

        self.get_logger().info(f'Publishing: {self.angle_deg:.1f} deg')

        self.angle_deg += self.step_deg
        if self.angle_deg > self.max_deg:
            self.angle_deg = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = DeltaJointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()