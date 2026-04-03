#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import json
import struct
import threading
import signal
from typing import Dict, Optional, Tuple

from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
from fk_ik import delta_calcInverse, delta_calcForward


# =========================================================
# DELTA GEOMETRY (mm)  — MUST match fk_ik.py
# =========================================================
e  = 35.0
f  = 157.0
re = 400.0
rf = 200.0

# =========================================================
# CONTROL CONFIG
# =========================================================
CHANNEL = "can0"
CONTROL_HZ = 50.0
RX_HZ = 50.0

KP = 15.0
KD = 5.0

MOTOR_IDS = [1, 2, 3]


# =========================================================
# FK SAFETY
# =========================================================
FK_ERROR_THRESHOLD_MM = 80

FK_ERROR_CONSECUTIVE_LIMIT = 5


# =========================================================
# FIND POSITION PARAMETER (SDK ADAPTIVE)
# =========================================================
def find_position_parameter():
    candidates = []
    for name in dir(ParameterType):
        if name.startswith("loc_ref"):
            continue
        lname = name.lower()
        if "pos" in lname and "kp" not in lname and "kd" not in lname:
            candidates.append(name)

    if not candidates:
        raise RuntimeError("❌ No POSITION-like parameter found in ParameterType")

    for p in ["POSITION", "POS", "JOINT_POSITION", "Q"]:
        if p in candidates:
            return getattr(ParameterType, p)

    return getattr(ParameterType, candidates[0])


# =========================================================
# CONTROLLER
# =========================================================
class DeltaIKMitFkController:
    def __init__(self):
        self.running = True
        self.lock = threading.Lock()

        # Motors
        self.motors: Dict[str, Motor] = {
            f"motor_{i}": Motor(id=i, model="rs-00")
            for i in MOTOR_IDS
        }

        self.calibration = {
            name: {"direction": 1, "homing_offset": 0.0}
            for name in self.motors
        }

        # Joint targets & feedback
        self.targets_rad = {name: 0.0 for name in self.motors}
        self.feedback_rad = {name: 0.0 for name in self.motors}
        self.feedback_ok  = {name: False for name in self.motors}
        self.feedback_current = {name: None for name in self.motors}  # Add dictionary for motor currents

        # Last commanded XYZ
        self.last_xyz: Optional[Tuple[float, float, float]] = None

        # FK safety
        self.fk_bad_count = 0

        # Discover POSITION parameter
        self.POSITION_PARAM = find_position_parameter()
        self.POS_ID, _, _ = self.POSITION_PARAM
        print(f"✅ Using position parameter: {self.POSITION_PARAM}")

        # CAN bus
        self.bus = RobstrideBus(CHANNEL, self.motors, self.calibration)

    # -----------------------------------------------------
    # RAW MODE SWITCH → MIT MODE (Mode 0)
    # -----------------------------------------------------
    def _set_mode_raw(self, motor_name: str):
        device_id = self.bus.motors[motor_name].id
        param_id, _, _ = ParameterType.MODE

        data = struct.pack("<HHbBH", param_id, 0, 0, 0, 0)
        self.bus.transmit(
            CommunicationType.WRITE_PARAMETER,
            self.bus.host_id,
            device_id,
            data
        )

    # -----------------------------------------------------
    # CONNECT
    # -----------------------------------------------------
    def connect(self):
        print("🔍 Connecting CAN...")
        self.bus.connect(handshake=True)

        for name in self.motors:
            print(f"⚡ Enabling {name}")
            self.bus.enable(name)
            time.sleep(0.2)

            print(f"⚙️ {name}: MIT mode")
            self._set_mode_raw(name)

        # Start threads
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)

        self.tx_thread.start()
        self.rx_thread.start()

        print("✅ Controller running")

    # -----------------------------------------------------
    # TX LOOP (MIT STREAM)
    # -----------------------------------------------------
    def _tx_loop(self):
        while self.running:
            with self.lock:
                for name in self.motors:
                    self.bus.write_operation_frame(
                        name,
                        self.targets_rad[name],
                        KP, KD, 0.0, 0.0
                    )
            time.sleep(1.0 / CONTROL_HZ)

    # -----------------------------------------------------
    # RX LOOP (CACHE FEEDBACK ONLY)
    # -----------------------------------------------------
    def _rx_loop(self):
        while self.running:
            with self.lock:
                for name in self.motors:
                    device_id = self.bus.motors[name].id
                    try:
                        rep = self.bus.transmit(
                            CommunicationType.READ_PARAMETER,
                            self.bus.host_id,
                            device_id,
                            struct.pack("<HH", self.POS_ID, 0)
                        )
                    except Exception:
                        continue

                    pos = None
                    if isinstance(rep, dict):
                        pos = rep.get("value", None)
                    elif hasattr(rep, "value"):
                        pos = rep.value

                    if pos is not None:
                        self.feedback_rad[name] = float(pos)
                        self.feedback_ok[name] = True

                    # Query motor current (if available)
                    current = None
                    if isinstance(rep, dict):
                        current = rep.get("current", None)  # Assuming 'current' is the field name for motor current
                    elif hasattr(rep, "current"):
                        current = rep.current

                    if current is not None:
                        self.feedback_current[name] = current
                        print(f"Motor {name} current: {current} A")

            time.sleep(1.0 / RX_HZ)

    # -----------------------------------------------------
    # IK COMMAND
    # -----------------------------------------------------
    def set_xyz(self, x: float, y: float, z: float):
        status, t1, t2, t3 = delta_calcInverse(x, y, z, e, f, re, rf)
        if status != 0:
            print("❌ IK failed")
            return

        with self.lock:
            self.targets_rad["motor_1"] = math.radians(t1)
            self.targets_rad["motor_2"] = math.radians(t2)
            self.targets_rad["motor_3"] = math.radians(t3)
            self.last_xyz = (x, y, z)

        print(f"🎯 CMD xyz=({x},{y},{z}) → θ=[{t1:.2f},{t2:.2f},{t3:.2f}]")

    # -----------------------------------------------------
    # SINGLE-SHOT FEEDBACK + FK PRINT
    # -----------------------------------------------------
    def print_feedback_and_fk(self):
        with self.lock:
            if not all(self.feedback_ok.values()):
                print("❌ Feedback not available yet")
                return

            t1 = math.degrees(self.feedback_rad["motor_1"])
            t2 = math.degrees(self.feedback_rad["motor_2"])
            t3 = math.degrees(self.feedback_rad["motor_3"])

        print(f"θ feedback (deg): [{t1:.2f}, {t2:.2f}, {t3:.2f}]")

        status, x, y, z = delta_calcForward(t1, t2, t3, e, f, re, rf)

        if status != 0:
            print("❌ FK failed")
            return

        print(f"FK xyz (mm): ({x:.2f}, {y:.2f}, {z:.2f})")

        if self.last_xyz is not None:
            cx, cy, cz = self.last_xyz
            err = math.sqrt((x-cx)**2 + (y-cy)**2 + (z-cz)**2)
            print(f"FK error vs cmd: {err:.2f} mm")

    # -----------------------------------------------------
    # STOP
    # -----------------------------------------------------
    def stop(self):
        print("\n🛑 Stopping...")
        self.running = False
        time.sleep(0.2)

        for name in self.motors:
            try:
                self.bus.disable(name)
            except Exception:
                pass

        self.bus.disconnect()
        sys.exit(0)


# =========================================================
# MAIN
# =========================================================
def main():
    ctrl = DeltaIKMitFkController()

    def handler(sig, frame):
        ctrl.stop()

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    ctrl.connect()

    print("\nCommands:")
    print(" xyz x y z   → move end-effector")
    print(" fb          → print feedback + FK once")
    print(" q           → quit\n")

    while True:
        cmd = input(">> ").strip().lower()

        if cmd in ("q", "quit", "exit"):
            ctrl.stop()

        if cmd == "fb":
            ctrl.print_feedback_and_fk()
            continue

        parts = cmd.split()
        if len(parts) == 4 and parts[0] == "xyz":
            try:
                x, y, z = map(float, parts[1:])
                ctrl.set_xyz(x, y, z)
            except Exception:
                print("❌ Usage: xyz x y z")
            continue

        print("❌ Unknown command")


if __name__ == "__main__":
    main()
