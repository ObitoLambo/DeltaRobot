#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from PP43M import DeltaRobotController


class DeltaSubscriber(Node):

    def __init__(self):
        super().__init__('delta_simulink_subscriber')

        # use PP43M.py hardware controller
        self.controller = DeltaRobotController(channel='can0')

        if not self.controller.connect():
            self.get_logger().error("Delta controller connection failed")
            raise RuntimeError("Motor connection failed")

        # if Simulink sends radians -> keep True
        # if Simulink already sends degrees -> set False
        self.input_is_radian = True

        self.sub_1 = self.create_subscription(
            Float64,
            '/delta/joint_angle_1',
            self.cb_motor1,
            10
        )

        self.sub_2 = self.create_subscription(
            Float64,
            '/delta/joint_angle_2',
            self.cb_motor2,
            10
        )

        self.sub_3 = self.create_subscription(
            Float64,
            '/delta/joint_angle_3',
            self.cb_motor3,
            10
        )

        self.get_logger().info("✅ Delta Subscriber Ready")
        self.get_logger().info("Subscribed to /delta/joint_angle_1, _2, _3")

    def convert_to_deg(self, value):
        if self.input_is_radian:
            return math.degrees(value)
        return value

    def send_motor_command(self, motor_id, msg: Float64):
        angle_in = float(msg.data)
        angle_deg = self.convert_to_deg(angle_in)

        # PP43M uses absolute target angle in degree
        self.controller.set_angle(motor_id, angle_deg)

        # read latest feedback from controller thread
        current_rad = self.controller.feedback[motor_id]["position"]
        current_deg = math.degrees(current_rad)

        self.get_logger().info(
            f"Motor {motor_id} | Cmd: {angle_deg:.2f}° | Current: {current_deg:.2f}°"
        )

    def cb_motor1(self, msg: Float64):
        self.send_motor_command(1, msg)

    def cb_motor2(self, msg: Float64):
        self.send_motor_command(2, msg)

    def cb_motor3(self, msg: Float64):
        self.send_motor_command(3, msg)

    def destroy_node(self):
        # safer than calling stop_and_exit() directly inside ROS2 node
        try:
            self.controller.running = False

            if self.controller.control_thread:
                self.controller.control_thread.join(timeout=1.0)

            if self.controller.bus and self.controller.connected:
                with self.controller.lock:
                    self.get_logger().info("Returning all motors to 0°...")
                    for mid in [1, 2, 3]:
                        self.controller.bus.write_operation_frame(
                            self.controller.motor_names[mid],
                            0.0,
                            self.controller.kp,
                            self.controller.kd,
                            0.0,
                            0.0
                        )

                import time
                time.sleep(1.5)

                with self.controller.lock:
                    self.get_logger().info("Disabling all motors...")
                    for mid in [1, 2, 3]:
                        self.controller.bus.disable(self.controller.motor_names[mid])

                self.controller.bus.disconnect()

        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DeltaSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()