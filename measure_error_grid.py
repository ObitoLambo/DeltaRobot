#!/usr/bin/env python3
"""
Grid error map measurement script.

The robot holds a marker pen in the gripper.  Place a sheet of paper flat on
the belt surface (Z = MARK_Z_MM).  The script visits each grid point, lowers
the pen to mark the paper, then asks you to measure the offset between the dot
and the commanded position.

Result is saved to error_map.json.  Rebuild delta_common + delta_motor_controller,
set ERROR_MAP_ENABLE = True in config.py, and the motor controller will apply the
interpolated correction on every move.

Usage:
    python3 measure_error_grid.py [--dry-run]
"""
import sys
import os
import json
import time
import argparse

sys.path.insert(0, "/home/s4mb4th/delta_ws/install/delta_common/lib/python3.10/site-packages")
sys.path.insert(0, "/home/s4mb4th/delta_ws/install/delta_motor_controller/lib/python3.10/site-packages")

from delta_common import config
from delta_common.fk_ik import delta_calcInverse, e, f, re, rf, check_workspace
from delta_motor_controller.motor_controller import DeltaMotorController

# ── Grid settings ─────────────────────────────────────────────────────────────
GRID_X   = [-100, -50, 0, 50, 100]   # mm
GRID_Y   = [-100, -50, 0, 50, 100]   # mm
MARK_Z   = -410.0                     # paper surface (belt)
SAFE_Z   = -350.0                     # travel height (home)
HOLD_S   = 0.6                        # seconds pen stays on paper
OUTPUT   = "/home/s4mb4th/delta_ws/error_map.json"
# ──────────────────────────────────────────────────────────────────────────────


def build_grid():
    """Return grid points in boustrophedon (snake) order."""
    points = []
    for i, y in enumerate(GRID_Y):
        row = GRID_X if i % 2 == 0 else GRID_X[::-1]
        for x in row:
            points.append((float(x), float(y), MARK_Z))
    return points


def load_existing(path):
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        data = json.load(fh)
    done = {}
    for p in data.get("points", []):
        key = tuple(p["cmd_mm"])
        done[key] = p["error_mm"]
    return done


def save(path, done: dict):
    points = [{"cmd_mm": list(k), "error_mm": v} for k, v in done.items()]
    with open(path, "w") as fh:
        json.dump({"version": 1, "mark_z_mm": MARK_Z, "points": points}, fh, indent=2)


def get_float(prompt, default=0.0):
    while True:
        raw = input(prompt).strip()
        if raw == "":
            return default
        try:
            return float(raw)
        except ValueError:
            print("  Enter a number.")


def is_reachable(x, y, z):
    if not check_workspace(x, y, z):
        return False
    st, t1, t2, t3 = delta_calcInverse(x, y, z, e, f, re, rf)
    if st != 0:
        return False
    return (config.THETA1_MIN <= t1 <= config.THETA1_MAX and
            config.THETA2_MIN <= t2 <= config.THETA2_MAX and
            config.THETA3_MIN <= t3 <= config.THETA3_MAX)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true",
                        help="Print grid points without moving the robot")
    args = parser.parse_args()

    grid = build_grid()
    done = load_existing(OUTPUT)

    reachable = [(x, y, z) for x, y, z in grid if is_reachable(x, y, z)]
    skipped   = [(x, y, z) for x, y, z in grid if not is_reachable(x, y, z)]

    print(f"=== Grid Error Map Measurement ===")
    print(f"Grid: {len(GRID_X)}×{len(GRID_Y)} = {len(grid)} points  "
          f"| reachable: {len(reachable)}  | skipped (IK fail): {len(skipped)}")
    if skipped:
        print(f"  Skipped: {skipped}")
    print(f"Output: {OUTPUT}")
    print(f"Already measured: {len(done)} / {len(reachable)}\n")

    remaining = [pt for pt in reachable if tuple(pt) not in done]
    print(f"Remaining: {len(remaining)} points\n")

    if args.dry_run:
        for i, (x, y, z) in enumerate(reachable):
            status = "done" if tuple((x, y, z)) in done else "TODO"
            print(f"  {i+1:2d}. ({x:7.1f}, {y:7.1f}, {z:.1f})  [{status}]")
        return

    if not remaining:
        print("All points already measured.  Delete error_map.json to restart.")
        return

    # Force EE_OFFSET and ERROR_MAP off — measure raw kinematic error
    config.EE_OFFSET_X_MM = 0.0
    config.EE_OFFSET_Y_MM = 0.0
    config.EE_OFFSET_Z_MM = 0.0
    config.ERROR_MAP_ENABLE = False

    print("NOTE: EE_OFFSET and ERROR_MAP disabled for this session (raw measurement).\n")
    print("Place a flat sheet of paper on the belt surface at Z =", MARK_Z, "mm.")
    input("Press Enter when ready to start...\n")

    ctrl = DeltaMotorController(can_port="can0")
    ctrl.connect()
    print("Motors connected. Moving to HOME...\n")
    ctrl.move_xyz(0.0, 0.0, SAFE_Z, raw=True)

    total = len(remaining)
    for idx, (x, y, z) in enumerate(remaining):
        key = (x, y, z)
        print(f"─── Point {idx+1}/{total}: commanded ({x:.0f}, {y:.0f}, {z:.0f}) mm ───")

        # Approach: move XY at safe height, then descend
        ctrl.move_xyz(x, y, SAFE_Z)
        ctrl.move_xyz(x, y, MARK_Z)
        time.sleep(HOLD_S)
        ctrl.move_xyz(x, y, SAFE_Z)

        print(f"  Dot made at commanded ({x:.0f}, {y:.0f}) mm.")
        print("  Measure where the dot actually landed on the paper.")
        print("  Enter  actual_dot_x - commanded_x  (positive = dot is further in +X).")
        print("  Leave blank = 0 mm\n")

        ex = get_float(f"  Error X mm [0]: ", 0.0)
        ey = get_float(f"  Error Y mm [0]: ", 0.0)
        ez = get_float(f"  Error Z mm [0]: ", 0.0)

        done[key] = [ex, ey, ez]
        save(OUTPUT, done)
        print(f"  Saved: error=({ex:+.1f}, {ey:+.1f}, {ez:+.1f}) mm\n")

    print("Moving to HOME...")
    ctrl.move_xyz(0.0, 0.0, SAFE_Z, raw=True)
    ctrl.shutdown()

    print(f"\n=== Measurement complete ===")
    print(f"{len(done)} points saved to {OUTPUT}")
    print("\nNext steps:")
    print("  1. colcon build --packages-select delta_common delta_motor_controller")
    print("  2. Set ERROR_MAP_ENABLE = True  in config.py")
    print("  3. Set EE_OFFSET_X/Y/Z = 0.0   in config.py")
    print("  4. Rebuild again, then test with tune_ee_offset.py")


if __name__ == "__main__":
    main()
