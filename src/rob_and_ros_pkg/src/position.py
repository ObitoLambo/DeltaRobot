#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride Position Mode Control (Mode 1)
Mode: Mode 1 (Position Mode)
Communication: Call write_operation_frame in a loop

Usage: python3 position_control.py <motor_id>
"""

import sys
import os
import time
import math
import struct
import threading
import signal
from typing import Optional

# Try to import the SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
except ImportError:
    # Assuming the current directory structure
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType, CommunicationType
    except ImportError as e:
        print(f"❌ Unable to import SDK: {e}")
        sys.exit(1)


class PositionController:
    def __init__(self, motor_id: int, channel='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel

        
        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()  # Mutex lock to prevent socket conflict

        self.running = True
        self.connected = False
        self.target_position = 0.0  # Target position (rad)

        # Default parameters for Position Mode
        self.kp = 100.0  # Stiffness (Nm/rad) [optional, in case needed]
        self.kd = 5.0  # Damping (Nm/rad/s) [optional, in case needed]

        self.position=0
        self.velocity=0
        self.torque=0
        self.temperature=0

    def _signal_handler(self, signum, frame):
        self.stop_and_exit()

    def _set_mode_raw(self, mode: int):
        """
        Set mode using raw transmit to send the mode switch command without waiting for a response (to avoid connect timeout)
        """
        print(f"⚙️ Switching to Position Mode (Mode {mode})")
        device_id = self.bus.motors[self.motor_name].id
        param_id, param_dtype, _ = ParameterType.MODE

        # MODE is int8
        value_buffer = struct.pack("<bBH", mode, 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer

        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)
        time.sleep(0.1)  # Wait for mode switch
        print(f"✅ Mode switched to Position Mode")

    def feedback_frame(self):
        print(f"Current position: {math.degrees(self.position):.2f}°")
        print(f"Current velocity: {(self.velocity):.2f} rad/s")
        print(f"Current torque: {(self.torque):.2f} n/m")
        print(f"Current temperature: {(self.temperature):.2f} C°")
        return True

        

    def connect(self):
        print(f"🔍 Connecting to CAN channel {self.channel}...")

        # Define motors
        motors = {
            self.motor_name: Motor(id=self.motor_id, model="rs-00")  # Modify model as needed
        }

        # Simple calibration parameters
        calibration = {
            self.motor_name: {"direction": 1, "homing_offset": 0.0}
        }
    
        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)

            with self.lock:
                # Activate the motor
                print(f"⚡ Activating motor ID: {self.motor_id} ...")
                self.bus.enable(self.motor_name)
                time.sleep(0.5)

                # *********************
                # *** Core Logic ***
                # *********************
                # 1. Switch to Position Mode (Mode 1)
                self._set_mode_raw(1)  # Switching to Position Mode

                # 2. Set a known, safe initial target position (0.0)
                self.initial_position = 0.0
                self.target_position = math.radians(self.initial_position)  # Set to 0 radians

                # 3. Send the first position control frame
                self.bus.write_operation_frame(
                    self.motor_name,
                    self.target_position,
                    self.kp,
                    self.kd,
                    0.0,  # velocity_ff
                    0.0  # torque_ff
                )
                print(f"🏠 Initial target set to: 0.0°")
                # 4. Read the first frame
                self.position, self.velocity, self.torque, self.temperature = self.bus.read_operation_frame(self.motor_name)
                print(f"Current position: {math.degrees(self.position):.2f}°")
                print(f"Current velocity: {(self.velocity):.2f} rad/s")
                print(f"Current torque: {(self.torque):.2f} n/m")
                print(f"Current temperature: {(self.temperature):.2f} C°")
            self.connected = True

            # Start background control thread
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
                    # Send position control frame
                    self.bus.write_operation_frame(
                        self.motor_name,
                        self.target_position,
                        self.kp,
                        self.kd,
                        0.0,  # velocity_ff
                        0.0  # torque_ff
                    )

                    # Read status frame (to clear the CAN receive buffer)
                    self.position, self.velocity, self.torque, self.temperature = self.bus.read_operation_frame(self.motor_name)
                    #print(f"Current position: {math.degrees(position):.2f}°")
                    #print(f"Current velocity: {math.degrees(velocity):.2f}°")
                    #print(f"Current torque: {math.degrees(torque):.2f}°")
                    #print(f"Current temperature: {math.degrees(temperature):.2f}°")
                time.sleep(0.02)  # 50Hz control frequency

            except Exception as e:
                if "No response from the motor" not in str(e):
                    print(f"⚠️ Communication error: {e}")
                time.sleep(0.5)

    def set_angle(self, angle_degrees: float):
        """Set target angle (in degrees)"""
        # Limit the range, for example +/- 2 turns
        angle_degrees = max(-10.0, min(200.0, angle_degrees))
        self.target_position = math.radians(angle_degrees)
        print(f" -> Target set to: {angle_degrees:.1f}°")

    def stop_and_exit(self):
        print("\n🛑 Stopping...")
        self.running = False

        if self.control_thread:
            self.control_thread.join(timeout=0.5)  # Wait for thread to exit

        if self.bus and self.connected:
            try:
                with self.lock:
                    # Return to zero position
                    print("🏠 Returning to zero position...")
                    self.bus.write_operation_frame(self.motor_name, 0.0, self.kp, self.kd, 0.0, 0.0)
                    time.sleep(1.0)  # Wait for motor to move
                    # Disable motor
                    print("🚫 Disabling motor...")
                    self.bus.disable(self.motor_name)
            except Exception as e:
                print(f"⚠️ Error while stopping: {e}")
            finally:
                self.bus.disconnect()

        print("👋 Program terminated")
        sys.exit(0)

    def run_interactive(self):
        print("\n" + "=" * 40)
        print(f"🎮 Position Mode Control Console (ID: {self.motor_id})")
        print("=" * 40)
        
        print("👉 Directly input a number (in degrees) to change position")
        print("👉 'fb' to to read CurPos, Vel, Torque and Temp")
        print("👉 '0' or 'home' to return to zero position")
        print("👉 'q' to quit")
        print("-" * 40)

        while True:
            try:
                cmd = input(f"[{math.degrees(self.target_position):.1f}°] >> ").strip().lower()

                if not cmd:
                    continue

                if cmd in ['q', 'quit', 'exit']:
                    break

                if cmd in ['0', 'home']:
                    self.set_angle(0.0)
                    continue

                if cmd in ['fb']:
                    self.feedback_frame()
                    continue

                try:
                    angle = float(cmd)
                    self.set_angle(angle)
                except ValueError:
                    print("❌ Invalid input, please enter a number (angle) or 'home', 'q'")

            except KeyboardInterrupt:
                break

        self.stop_and_exit()


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 position_control.py <motor_id>")
        sys.exit(1)

    motor_id = int(sys.argv[1])

    controller = PositionController(motor_id)
    signal.signal(signal.SIGINT, controller._signal_handler)
    signal.signal(signal.SIGTERM, controller._signal_handler)

    if controller.connect():
        controller.run_interactive()


if __name__ == "__main__":
    main()
