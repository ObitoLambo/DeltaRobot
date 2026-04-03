#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import math
import struct
import threading
import signal
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# Try to import the SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
except ImportError:
    from bus import RobstrideBus, Motor
    from protocol import ParameterType, CommunicationType


# =====================================================
# RobStride Position Controller (same style as yours)
# =====================================================

class PositionController:
    def __init__(self, motor_id: int, channel='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel

        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()

        self.running = True
        self.connected = False

        # targets
        self.target_position = 0.0      # final command (rad)
        self.zero_position = 0.0        # reference (rad), captured at enable

        # gains (stronger so it actually moves)
        self.kp = 400.0
        self.kd = 20.0

        # feedback
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.temperature = 0.0
        self.angle_deg = 0.0
        self.control_thread: Optional[threading.Thread] = None

    def _signal_handler(self, signum, frame):
        self.stop_and_exit()

    def _set_mode_raw(self, mode: int):
        print(f"⚙️ Switching to Position Mode (Mode {mode})")
        device_id = self.bus.motors[self.motor_name].id
        param_id, _, _ = ParameterType.MODE

        # MODE is int8
        value_buffer = struct.pack("<bBH", mode, 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer

        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)
        time.sleep(0.2)
        print("✅ Mode switched")

    def connect(self):
        print(f"🔍 Connecting to CAN channel {self.channel}...")

        motors = {self.motor_name: Motor(id=self.motor_id, model="rs-00")}
        calibration = {self.motor_name: {"direction": 1, "homing_offset": 0.0}}

        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            # if handshake is flaky, you can set False:
            self.bus.connect(handshake=True)

            with self.lock:
                print(f"⚡ Activating motor ID: {self.motor_id} ...")
                self.bus.enable(self.motor_name)
                time.sleep(0.8)

                # switch mode
                self._set_mode_raw(1)

                # IMPORTANT: read current position and lock it as reference
                self.position, self.velocity, self.torque, self.temperature = \
                    self.bus.read_operation_frame(self.motor_name)

                self.zero_position = self.position
                self.target_position = self.zero_position

                print(f"🏠 Zero position captured: {math.degrees(self.zero_position):.2f}°")

                # send a few frames to "latch" the controller
                for _ in range(10):
                    self.bus.write_operation_frame(
                        self.motor_name,
                        self.target_position,
                        self.kp,
                        self.kd,
                        0.0,
                        0.0
                    )
                    time.sleep(0.02)

            self.connected = True

            self.control_thread = threading.Thread(target=self.loop, daemon=True)
            self.control_thread.start()

            print("✅ Initialization complete (Position Mode)!")
            return True

        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.connected = False
            return False

    def loop(self):
        """Control loop: continuously send position control frames"""
        print("🔄 Control loop started (Position Mode @ 50Hz)")
        while self.running and self.connected:
            try:
                with self.lock:
                    self.bus.write_operation_frame(
                        self.motor_name,
                        self.target_position,
                        self.kp,
                        self.kd,
                        0.0,
                        0.0
                    )
                    # read feedback each cycle (like your original)
                    self.position, self.velocity, self.torque, self.temperature = \
                        self.bus.read_operation_frame(self.motor_name)

                time.sleep(0.02)

            except Exception as e:
                # ignore occasional no-response hiccups
                time.sleep(0.05)

    # --------- THIS IS THE KEY FIX ----------
    def set_angle_relative(self, angle_degrees: float):
        """
        Interpret ROS input as RELATIVE motion from the captured zero.
        Example: 100 means 'move +100° from where you were when enabled'
        """
        angle_degrees = max(-180.0, min(180.0, float(angle_degrees)))
        with self.lock:
                self.target_position = self.zero_position + math.radians(angle_degrees)
                self.bus.write_operation_frame(
                        self.motor_name,
                        self.target_position,
                        self.kp,
                        self.kd,
                        0.0,
                        0.0
                )
           
        print(f"🎯 New REL target: {angle_degrees:.1f}° (abs: {math.degrees(self.target_position):.1f}°)")

    def stop_and_exit(self):
        print("\n🛑 Stopping...")
        self.running = False

        if self.control_thread:
            self.control_thread.join(timeout=0.5)

        if self.bus and self.connected:
            try:
                with self.lock:
                    print("🚫 Disabling motor...")
                    self.bus.disable(self.motor_name)
            except Exception:
                pass
            finally:
                try:
                    self.bus.disconnect()
                except Exception:
                    pass

        print("👋 Program terminated")
        sys.exit(0)


# =====================================================
# ROS2 Node Wrapper
# =====================================================

class RobRosNode(Node):
    def __init__(self, motor_id: int):
        super().__init__('rob_n_ros_node')

        self.controller = PositionController(motor_id)

        if not self.controller.connect():
            self.get_logger().error("Motor connection failed")
            # don’t call rclpy.shutdown here; let main handle it
            return

        self.subscription = self.create_subscription(
            Float64,
            '/simulink_topic',
            self.listener_callback,
            10
        )

        self.get_logger().info("🚀 ROS Motor Node Ready (Relative Command)")

    def listener_callback(self, msg: Float64):
        self.angle_deg = msg.data
        self.controller.set_angle_relative(self.angle_deg)

        self.get_logger().info(
            f"TargetRel: {float(self.angle_deg):.2f}° | "
            f"Current: {math.degrees(self.controller.position):.2f}° | "
            f"Torque: {self.controller.torque:.3f}"
        )

    def destroy_node(self):
        # Clean stop without sys.exit in ROS teardown path
        try:
            self.controller.running = False
            if self.controller.control_thread:
                self.controller.control_thread.join(timeout=0.5)
            if self.controller.bus and self.controller.connected:
                with self.controller.lock:
                    self.controller.bus.disable(self.controller.motor_name)
                self.controller.bus.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run rob_and_ros_pkg rob_n_ros_node <motor_id>")
        return

    motor_id = int(sys.argv[1])

    rclpy.init(args=args)
    node = RobRosNode(motor_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
