#!/usr/bin/env python3
"""
Grid error map measurement — camera-based (automatic).

The camera detects the white EE marker on the gripper tip via
/delta/ee_position_mm.  The script sweeps a 9×9 grid, moves the
robot to each point, waits for it to settle, then averages 10
EE-position samples from the camera to compute the landing error.

Result is saved to error_map.json.  Set ERROR_MAP_ENABLE = True
in config.py to activate correction on every move.

Usage:
    python3 measure_error_grid.py [--dry-run]
"""
import sys
import os
import json
import time
import argparse
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

sys.path.insert(0, "/home/s4mb4th/delta_ws/install/delta_common/lib/python3.10/site-packages")
sys.path.insert(0, "/home/s4mb4th/delta_ws/install/delta_motor_controller/lib/python3.10/site-packages")

from delta_common import config
from delta_common.fk_ik import delta_calcInverse, e, f, re, rf, check_workspace
from delta_motor_controller.motor_controller import DeltaMotorController

# ── Grid settings ──────────────────────────────────────────────────────────────
GRID_X   = list(range(-120, 121, 30))   # 9 points: -120 … +120 mm
GRID_Y   = list(range(-120, 121, 30))   # 9 points: -120 … +120 mm
MARK_Z   = -410.0                        # belt surface Z (mm)
SAFE_Z   = -350.0                        # travel height
SETTLE_S = 1.0                           # seconds to wait after move
SAMPLES  = 10                            # EE position samples to average
SAMPLE_WINDOW_S = 1.0                    # collection window per point
OUTPUT   = "/home/s4mb4th/delta_ws/error_map.json"
# ──────────────────────────────────────────────────────────────────────────────


class EEListener(Node):
    def __init__(self):
        super().__init__("ee_listener")
        self._buf: deque = deque(maxlen=50)
        self._lock = threading.Lock()
        self.create_subscription(
            PointStamped, "/delta/ee_position_mm", self._cb, 10
        )

    def _cb(self, msg: PointStamped):
        with self._lock:
            self._buf.append((msg.point.x, msg.point.y, msg.point.z))

    def collect(self, n: int, window_s: float):
        """Spin for window_s seconds, return last n samples (or fewer)."""
        t_end = time.time() + window_s
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        with self._lock:
            samples = list(self._buf)[-n:]
        self._buf.clear()
        return samples


def build_grid():
    points = []
    for i, y in enumerate(GRID_Y):
        row = GRID_X if i % 2 == 0 else GRID_X[::-1]
        for x in row:
            points.append((float(x), float(y), MARK_Z))
    return points


def is_reachable(x, y, z):
    if not check_workspace(x, y, z):
        return False
    st, t1, t2, t3 = delta_calcInverse(x, y, z, e, f, re, rf)
    if st != 0:
        return False
    return (config.THETA1_MIN <= t1 <= config.THETA1_MAX and
            config.THETA2_MIN <= t2 <= config.THETA2_MAX and
            config.THETA3_MIN <= t3 <= config.THETA3_MAX)


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


def print_summary(done: dict):
    if not done:
        return
    errors = list(done.values())
    ex = [e[0] for e in errors]
    ey = [e[1] for e in errors]
    import math
    mean_x = sum(ex) / len(ex)
    mean_y = sum(ey) / len(ey)
    max_x  = max(abs(v) for v in ex)
    max_y  = max(abs(v) for v in ey)
    rms_x  = math.sqrt(sum(v**2 for v in ex) / len(ex))
    rms_y  = math.sqrt(sum(v**2 for v in ey) / len(ey))
    print(f"\n=== Error map summary ({len(done)} points) ===")
    print(f"  Mean  X={mean_x:+.2f}  Y={mean_y:+.2f} mm")
    print(f"  Max   X={max_x:.2f}    Y={max_y:.2f}    mm")
    print(f"  RMS   X={rms_x:.2f}    Y={rms_y:.2f}    mm")
    print(f"\nNext step: set ERROR_MAP_ENABLE = True in config.py")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    grid      = build_grid()
    done      = load_existing(OUTPUT)
    reachable = [(x, y, z) for x, y, z in grid if is_reachable(x, y, z)]
    skipped   = [(x, y, z) for x, y, z in grid if not is_reachable(x, y, z)]

    print(f"=== Camera-based Grid Error Map ===")
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
        print("All points measured.  Delete error_map.json to restart.")
        print_summary(done)
        return

    rclpy.init()
    listener = EEListener()

    config.EE_OFFSET_X_MM  = 0.0
    config.EE_OFFSET_Y_MM  = 0.0
    config.EE_OFFSET_Z_MM  = 0.0
    config.ERROR_MAP_ENABLE = False
    print("NOTE: EE_OFFSET and ERROR_MAP disabled for raw measurement.\n")

    ctrl = DeltaMotorController(can_port="can0")
    ctrl.connect()
    print("Motors connected. Moving to HOME...\n")
    ctrl.move_xyz(0.0, 0.0, SAFE_Z, raw=True)

    total = len(remaining)
    for idx, (x, y, z) in enumerate(remaining):
        key = (x, y, z)
        print(f"─── Point {idx+1}/{total}: commanded ({x:.0f}, {y:.0f}, {z:.0f}) mm ───")

        ctrl.move_xyz(x, y, SAFE_Z)
        ctrl.move_xyz(x, y, MARK_Z)
        time.sleep(SETTLE_S)

        samples = listener.collect(SAMPLES, SAMPLE_WINDOW_S)
        if not samples:
            print(f"  WARNING: no EE samples — skipping this point")
            ctrl.move_xyz(x, y, SAFE_Z)
            continue

        ex_vals = [s[0] for s in samples]
        ey_vals = [s[1] for s in samples]
        ex_act  = sum(ex_vals) / len(ex_vals)
        ey_act  = sum(ey_vals) / len(ey_vals)
        error_x = ex_act - x
        error_y = ey_act - y

        ctrl.move_xyz(x, y, SAFE_Z)

        print(f"  commanded=({x:.1f}, {y:.1f})  "
              f"actual=({ex_act:.1f}, {ey_act:.1f})  "
              f"error=({error_x:+.2f}, {error_y:+.2f}) mm  "
              f"[{len(samples)} samples]")

        done[key] = [error_x, error_y, 0.0]
        save(OUTPUT, done)

    print("\nMoving to HOME...")
    ctrl.move_xyz(0.0, 0.0, SAFE_Z, raw=True)
    ctrl.shutdown()
    listener.destroy_node()
    rclpy.shutdown()

    print_summary(done)


if __name__ == "__main__":
    main()
