#!/usr/bin/env python3
"""
EE offset tuning script.

Usage: python3 tune_ee_offset.py

Moves the robot to a test position, you measure where it actually lands,
enter the error, and the script applies the correction to config.py.

Convention: EE_OFFSET negates the observed error.
  If robot lands +10 mm in X from target -> set EE_OFFSET_X = -10.
"""
import sys
import re
import os

sys.path.insert(0, "/home/s4mb4th/delta_ws/install/delta_common/lib/python3.10/site-packages")
sys.path.insert(0, "/home/s4mb4th/delta_ws/install/delta_motor_controller/lib/python3.10/site-packages")

from delta_common import config
from delta_common.fk_ik import delta_calcInverse, e, f, re, rf
from delta_motor_controller.motor_controller import DeltaMotorController

CONFIG_PATH = "/home/s4mb4th/delta_ws/src/delta_common/delta_common/config.py"
HOME = (0.0, 0.0, -350.0)

# Default test target — well inside workspace, easy to measure
DEFAULT_TARGET = (0.0, 0.0, -410.0)


def update_config(key: str, value: float):
    with open(CONFIG_PATH, "r") as fh:
        text = fh.read()
    pattern = rf"^({re.escape(key)}\s*=\s*)[-\d.]+"
    new_line = rf"\g<1>{value:.2f}"
    updated = re.sub(pattern, new_line, text, flags=re.MULTILINE)
    if updated == text:
        print(f"  WARNING: could not find '{key}' in {CONFIG_PATH}")
        return
    with open(CONFIG_PATH, "w") as fh:
        fh.write(updated)
    print(f"  {key} = {value:.2f}  (written to config.py)")


def get_float(prompt, default=None):
    while True:
        raw = input(prompt).strip()
        if raw == "" and default is not None:
            return default
        try:
            return float(raw)
        except ValueError:
            print("  Enter a number.")


def main():
    print("=== EE Offset Tuning ===")
    print(f"Current offsets: X={config.EE_OFFSET_X_MM:.2f}  "
          f"Y={config.EE_OFFSET_Y_MM:.2f}  Z={config.EE_OFFSET_Z_MM:.2f}\n")

    ctrl = DeltaMotorController(can_port="can0")
    ctrl.connect()
    print("Motors connected.\n")

    print("Moving to HOME...")
    ctrl.move_xyz(*HOME, raw=True)

    tx = get_float(f"Test target X mm [{DEFAULT_TARGET[0]}]: ", DEFAULT_TARGET[0])
    ty = get_float(f"Test target Y mm [{DEFAULT_TARGET[1]}]: ", DEFAULT_TARGET[1])
    tz = get_float(f"Test target Z mm [{DEFAULT_TARGET[2]}]: ", DEFAULT_TARGET[2])

    print(f"\nMoving to test target ({tx}, {ty}, {tz}) mm ...")
    ok, ik, fb, fk_xyz, err = ctrl.move_xyz(tx, ty, tz)
    if fk_xyz is None:
        print("Move failed — check workspace limits or IK solution.")
        ctrl.shutdown()
        return

    print(f"FK reports: ({fk_xyz[0]:.2f}, {fk_xyz[1]:.2f}, {fk_xyz[2]:.2f})  err={err:.2f} mm")

    print("\nMeasure where the EEF actually landed relative to the TRUE target.")
    print("  actual_position = target + error  (positive = EEF is beyond target in that axis)")
    print("  Leave blank = 0 (no error in that axis)\n")

    ex = get_float("  Observed error in X mm (actual - target): ", 0.0)
    ey = get_float("  Observed error in Y mm (actual - target): ", 0.0)
    ez = get_float("  Observed error in Z mm (actual - target): ", 0.0)

    new_ox = config.EE_OFFSET_X_MM - ex
    new_oy = config.EE_OFFSET_Y_MM - ey
    new_oz = config.EE_OFFSET_Z_MM - ez

    print(f"\nNew offsets: X={new_ox:.2f}  Y={new_oy:.2f}  Z={new_oz:.2f}")
    confirm = input("Write to config.py? (y/n): ").strip().lower()
    if confirm == "y":
        update_config("EE_OFFSET_X_MM", new_ox)
        update_config("EE_OFFSET_Y_MM", new_oy)
        update_config("EE_OFFSET_Z_MM", new_oz)
        print("\nConfig updated. Rebuild the package and restart nodes to apply:\n"
              "  colcon build --packages-select delta_common && "
              "source install/setup.bash")
    else:
        print("No changes written.")

    print("\nReturning to HOME...")
    ctrl.move_xyz(*HOME, raw=True)
    ctrl.shutdown()


if __name__ == "__main__":
    main()
