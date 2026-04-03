#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride Position Mode Control - Delta Robot (3 Motors Parallel)
Motor IDs: 1, 2, 3
Communication: CAN Bus
"""

import sys
import os
import time
import math
import struct
import threading
import signal
from typing import Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
except ImportError:
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType, CommunicationType
    except ImportError as e:
        print(f"❌ Unable to import SDK: {e}")
        sys.exit(1)


MOTOR_IDS = [1, 2, 3]
CHANNEL = 'can0'
CONTROL_FREQ = 50  # Hz
ANGLE_MIN = -10.0
ANGLE_MAX = 200.0


class DeltaRobotController:
    def __init__(self, channel=CHANNEL):
        self.channel = channel
        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()

        self.running = True
        self.connected = False

        # Per-motor state
        self.motor_names = {mid: f"motor_{mid}" for mid in MOTOR_IDS}
        self.target_positions = {mid: 0.0 for mid in MOTOR_IDS}  # radians
        self.feedback = {
            mid: {"position": 0.0, "velocity": 0.0, "torque": 0.0, "temperature": 0.0}
            for mid in MOTOR_IDS
        }

        # PD gains
        self.kp = 100.0
        self.kd = 5.0

        self.control_thread: Optional[threading.Thread] = None

    # ─────────────────────────────────────────────
    # Internal helpers
    # ─────────────────────────────────────────────

    def _set_mode_raw(self, motor_name: str, motor_id: int, mode: int):
        """Send mode-switch command without waiting for a response."""
        param_id, _, _ = ParameterType.MODE
        value_buffer = struct.pack("<bBH", mode, 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        self.bus.transmit(
            CommunicationType.WRITE_PARAMETER,
            self.bus.host_id,
            self.bus.motors[motor_name].id,
            data
        )
        time.sleep(0.1)

    def _signal_handler(self, signum, frame):
        self.stop_and_exit()

    # ─────────────────────────────────────────────
    # Connect
    # ─────────────────────────────────────────────

    def connect(self) -> bool:
        print(f"🔍 Connecting to CAN channel: {self.channel} ...")

        motors = {
            self.motor_names[mid]: Motor(id=mid, model="rs-00")
            for mid in MOTOR_IDS
        }
        calibration = {
            self.motor_names[mid]: {"direction": 1, "homing_offset": 0.0}
            for mid in MOTOR_IDS
        }

        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)

            with self.lock:
                for mid in MOTOR_IDS:
                    name = self.motor_names[mid]
                    print(f"⚡ Enabling motor {mid} ...")
                    self.bus.enable(name)
                    time.sleep(0.3)

                    print(f"⚙️  Switching motor {mid} → Position Mode")
                    self._set_mode_raw(name, mid, 1)

                    # Send initial position frame (0°)
                    self.bus.write_operation_frame(
                        name, 0.0, self.kp, self.kd, 0.0, 0.0
                    )

                    # Read initial feedback
                    pos, vel, torq, temp = self.bus.read_operation_frame(name)
                    self.feedback[mid] = {
                        "position": pos, "velocity": vel,
                        "torque": torq, "temperature": temp
                    }
                    print(
                        f"   Motor {mid} → "
                        f"pos={math.degrees(pos):.2f}°  "
                        f"vel={vel:.2f} rad/s  "
                        f"torque={torq:.2f} Nm  "
                        f"temp={temp:.1f}°C"
                    )

            self.connected = True

            # Start the shared control loop thread
            self.control_thread = threading.Thread(
                target=self._control_loop, daemon=True
            )
            self.control_thread.start()

            print("✅ Delta robot initialized — all 3 motors in Position Mode!\n")
            return True

        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.connected = False
            return False

    # ─────────────────────────────────────────────
    # Control loop (single thread, all 3 motors)
    # ─────────────────────────────────────────────

    def _control_loop(self):
        """Send position frames to all 3 motors at CONTROL_FREQ Hz."""
        print(f"🔄 Control loop started ({CONTROL_FREQ}Hz, 3 motors)")
        interval = 1.0 / CONTROL_FREQ

        while self.running and self.connected:
            try:
                with self.lock:
                    for mid in MOTOR_IDS:
                        name = self.motor_names[mid]

                        self.bus.write_operation_frame(
                            name,
                            self.target_positions[mid],
                            self.kp,
                            self.kd,
                            0.0,  # velocity_ff
                            0.0   # torque_ff
                        )

                        pos, vel, torq, temp = self.bus.read_operation_frame(name)
                        self.feedback[mid] = {
                            "position": pos, "velocity": vel,
                            "torque": torq, "temperature": temp
                        }

                time.sleep(interval)

            except Exception as e:
                if "No response from the motor" not in str(e):
                    print(f"⚠️  Control loop error: {e}")
                time.sleep(0.5)

    # ─────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────

    def set_angle(self, motor_id: int, angle_degrees: float):
        """Set target angle for a single motor (degrees)."""
        if motor_id not in MOTOR_IDS:
            print(f"❌ Invalid motor ID: {motor_id}")
            return
        angle_degrees = max(ANGLE_MIN, min(ANGLE_MAX, angle_degrees))
        self.target_positions[motor_id] = math.radians(angle_degrees)
        print(f"   Motor {motor_id} → target: {angle_degrees:.1f}°")

    def set_all_angles(self, angles: Tuple[float, float, float]):
        """Set target angles for all 3 motors at once (degrees)."""
        for mid, angle in zip(MOTOR_IDS, angles):
            self.set_angle(mid, angle)

    def print_feedback(self, motor_id: Optional[int] = None):
        """Print feedback for one or all motors."""
        ids = [motor_id] if motor_id else MOTOR_IDS
        print("-" * 50)
        for mid in ids:
            fb = self.feedback[mid]
            print(
                f"  Motor {mid} | "
                f"pos={math.degrees(fb['position']):7.2f}°  "
                f"vel={fb['velocity']:6.2f} rad/s  "
                f"torque={fb['torque']:6.2f} Nm  "
                f"temp={fb['temperature']:5.1f}°C"
            )
        print("-" * 50)

    # ─────────────────────────────────────────────
    # Stop
    # ─────────────────────────────────────────────

    def stop_and_exit(self):
        print("\n🛑 Stopping delta robot...")
        self.running = False

        if self.control_thread:
            self.control_thread.join(timeout=1.0)

        if self.bus and self.connected:
            try:
                with self.lock:
                    print("🏠 Returning all motors to 0° ...")
                    for mid in MOTOR_IDS:
                        self.bus.write_operation_frame(
                            self.motor_names[mid], 0.0,
                            self.kp, self.kd, 0.0, 0.0
                        )
                    time.sleep(1.5)

                    print("🚫 Disabling all motors ...")
                    for mid in MOTOR_IDS:
                        self.bus.disable(self.motor_names[mid])
            except Exception as e:
                print(f"⚠️  Error while stopping: {e}")
            finally:
                self.bus.disconnect()

        print("👋 Delta robot shut down.")
        sys.exit(0)

    # ─────────────────────────────────────────────
    # Interactive console
    # ─────────────────────────────────────────────

    def run_interactive(self):
        print("\n" + "=" * 50)
        print("🤖 Delta Robot Position Control Console")
        print("=" * 50)
        print("Commands:")
        print("  <id> <angle>     → e.g. '1 45' sets motor 1 to 45°")
        print("  all <a1> <a2> <a3> → e.g. 'all 30 45 60'")
        print("  fb               → feedback from all motors")
        print("  fb <id>          → feedback from one motor")
        print("  home             → all motors to 0°")
        print("  q                → quit")
        print("-" * 50)

        while True:
            try:
                # Show current targets in prompt
                targets = "  ".join(
                    f"M{mid}={math.degrees(self.target_positions[mid]):.1f}°"
                    for mid in MOTOR_IDS
                )
                cmd = input(f"[{targets}] >> ").strip().lower()

                if not cmd:
                    continue

                # ── quit ──
                if cmd in ['q', 'quit', 'exit']:
                    break

                # ── home ──
                if cmd == 'home':
                    self.set_all_angles((0.0, 0.0, 0.0))
                    continue

                # ── feedback ──
                if cmd == 'fb':
                    self.print_feedback()
                    continue

                parts = cmd.split()

                if parts == 'fb' and len(parts) == 2:
                    try:
                        self.print_feedback(int(parts))
                    except ValueError:
                        print("❌ Usage: fb <motor_id>")
                    continue

                # ── all <a1> <a2> <a3> ──
                if parts == 'all' and len(parts) == 4:
                    try:
                        angles = tuple(float(p) for p in parts[1:])
                        self.set_all_angles(angles)
                    except ValueError:
                        print("❌ Usage: all <angle1> <angle2> <angle3>")
                    continue

                # ── <id> <angle> ──
                if len(parts) == 2:
                    try:
                        mid = int(parts)
                        angle = float(parts)
                        self.set_angle(mid, angle)
                    except ValueError:
                        print("❌ Usage: <motor_id> <angle>")
                    continue

                print("❌ Unknown command. Type 'q' to quit.")

            except KeyboardInterrupt:
                break

        self.stop_and_exit()


# ─────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────

def main():
    controller = DeltaRobotController()
    signal.signal(signal.SIGINT, controller._signal_handler)
    signal.signal(signal.SIGTERM, controller._signal_handler)

    if controller.connect():
        controller.run_interactive()


if __name__ == "__main__":
    main()