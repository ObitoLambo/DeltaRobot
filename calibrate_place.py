#!/usr/bin/env python3
"""
Place position calibration script.
Moves the robot to candidate positions so you can verify visually.
Run: python3 calibrate_place.py
"""
import sys
import os
sys.path.insert(0, "/home/s4mb4th/delta_ws/install/delta_common/lib/python3.10/site-packages")
sys.path.insert(0, "/home/s4mb4th/delta_ws/install/delta_motor_controller/lib/python3.10/site-packages")

from delta_common import config
from delta_common.fk_ik import delta_calcInverse, delta_calcForward, e, f, re, rf
from delta_motor_controller.motor_controller import DeltaMotorController

HOME_X, HOME_Y, HOME_Z = 0.0, 0.0, -350.0

def try_position(ctrl, x, y, z):
    from delta_common.fk_ik import check_workspace
    from delta_common.fk_ik import delta_calcInverse, e, f, re, rf
    st, t1, t2, t3 = delta_calcInverse(x, y, z, e, f, re, rf)
    if st != 0:
        print(f"  IK no solution for ({x}, {y}, {z})")
        return False
    if not (config.THETA1_MIN <= t1 <= config.THETA1_MAX and
            config.THETA2_MIN <= t2 <= config.THETA2_MAX and
            config.THETA3_MIN <= t3 <= config.THETA3_MAX):
        print(f"  Joint limit exceeded: theta=({t1:.1f}, {t2:.1f}, {t3:.1f})")
        return False
    ok, ik_deg, fb_deg, fk_xyz, err = ctrl.move_xyz(x, y, z)
    if ok and fk_xyz:
        print(f"  Moved to ({fk_xyz[0]:.1f}, {fk_xyz[1]:.1f}, {fk_xyz[2]:.1f}) mm  err={err:.2f} mm")
        return True
    return False


def main():
    print("=== Place Position Calibration ===")
    print("This script moves the robot so you can find the right drop position.")
    print("Press Ctrl+C at any time to abort and return to HOME.\n")

    ctrl = DeltaMotorController(can_port="can0")
    ctrl.connect()
    print("Motors connected.\n")

    # Start at HOME
    print("Moving to HOME (0, 0, -350)...")
    ctrl.move_xyz(HOME_X, HOME_Y, HOME_Z, raw=True)

    x, y, z = config.PLACE_X, config.PLACE_Y, config.PLACE_Z
    print(f"\nCurrent PLACE config: ({x}, {y}, {z}) mm")
    print("Moving to current place position...")
    try_position(ctrl, x, y, z)

    print("\nEnter new coordinates to test. Leave blank to keep current value.")
    print("Type 'done' to finish and print the config line.\n")

    while True:
        try:
            cmd = input("  x y z  (or 'home' / 'done'): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == "done":
            break
        if cmd == "home":
            ctrl.move_xyz(HOME_X, HOME_Y, HOME_Z)
            print("  At HOME.")
            continue
        if not cmd:
            continue

        parts = cmd.split()
        if len(parts) != 3:
            print("  Enter exactly 3 values: x y z")
            continue
        try:
            nx, ny, nz = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("  Invalid numbers.")
            continue

        if try_position(ctrl, nx, ny, nz):
            x, y, z = nx, ny, nz
            confirm = input("  Looks good? (y/n): ").strip().lower()
            if confirm == "y":
                print(f"\n  Confirmed: PLACE = ({x}, {y}, {z})")

    print("\nReturning to HOME...")
    ctrl.move_xyz(HOME_X, HOME_Y, HOME_Z, raw=True)
    ctrl.bus.disable_all()

    print("\n=== Calibration result ===")
    print(f"Add these lines to config.py:\n")
    print(f"  PLACE_X = {x}")
    print(f"  PLACE_Y = {y}")
    print(f"  PLACE_Z = {z}")


if __name__ == "__main__":
    main()
