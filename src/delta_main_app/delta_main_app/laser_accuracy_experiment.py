#!/usr/bin/env python3
"""
laser_accuracy_experiment.py — Delta Robot X/Y Accuracy & Repeatability Test

Methodology follows: Yamtuan et al., "Visual Servo Kinematic Control of Delta
Robot using YOLOv5 Algorithm," JRC Vol.4 No.6 2023 — Section III
(laser-pointer accuracy & repeatability experiment, Figs 10-15, Tables III-V).

Camera setup
------------
The D455 is mounted externally (fixed to the frame, NOT on the robot).
A homography calibration is run at startup: the robot moves to 4 known
positions and the operator captures the laser dot pixel at each position.
The resulting H matrix maps any pixel (u,v) → robot mm (x,y) directly,
with no knowledge of camera pose, orientation, or intrinsics required.

Physical setup before running
------------------------------
    1. Mount a laser pointer on the end-effector pointing straight down.
    2. Tape paper on the workspace floor.  Draw reference X and Y axes on it
       (matching the paper's Fig. 10).  The paper surface should be close to
       EXPERIMENT_Z_MM below the robot base.
    3. Make sure the D455 has a clear view of the whole paper area.
    4. Run the node.  The calibration phase guides you through 4 positions.

Error convention (paper Eq. 6)
-------------------------------
    P_error_j = P_j,command − P_j,measured
    Positive = robot stopped short.  Negative = robot overshot.

Outputs
-------
    ~/delta_ws/experiment_results/laser_accuracy_<timestamp>.csv
    ~/delta_ws/experiment_results/laser_accuracy_<timestamp>.png

Usage
-----
    ros2 run delta_main_app laser_accuracy_experiment
    # encoder-only (no camera, FK feedback only):
    ros2 run delta_main_app laser_accuracy_experiment --ros-args -p laser_detect:=false
"""

import csv
import math
import os
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import rclpy.parameter
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from delta_common import config
from delta_common.fk_ik import check_workspace
from delta_motor_controller.motor_controller import DeltaMotorController

# ── Experiment parameters — edit these before running ─────────────────────────

# Fixed Z for the whole experiment (mm, robot base frame).
# Adjust so the end-effector is just above the paper surface.
EXPERIMENT_Z_MM: float = -350.0

# Homography calibration positions (x, y) in mm.
# These 4 points must be non-collinear and span a reasonable area.
# The robot visits each in order during the calibration phase.
CALIB_POSITIONS_MM: List[Tuple[float, float]] = [
    (   0.0,   0.0),   # origin / home
    ( 100.0,   0.0),   # +X
    ( 100.0, 100.0),   # +X+Y
    (   0.0, 100.0),   # +Y
]

# Accuracy test positions (x, y) in mm — paper's 8-position layout (Table III).
# Pure-axis moves first, then diagonals.
TEST_GRID_MM: List[Tuple[float, float]] = [
    (   0.0,   0.0),   # home / reference
    ( 120.0,   0.0),   # +X
    (-120.0,   0.0),   # -X
    (   0.0, 120.0),   # +Y
    (   0.0,-120.0),   # -Y
    (  60.0,  60.0),   # +X+Y
    ( -60.0,  60.0),   # -X+Y
    (  60.0, -60.0),   # +X-Y
    ( -60.0, -60.0),   # -X-Y
]

N_REPEATS: int = 6          # repeats per point (paper uses 6, Fig. 14-15)
SETTLE_TIME_S: float = 1.0  # wait after motion before capturing laser dot
CAPTURE_FRAMES: int = 7     # frames averaged per laser-dot measurement

LASER_COLOR: str = "red"    # "red" or "green"

OUTPUT_DIR: str = os.path.expanduser("~/delta_ws/experiment_results")

# ── Laser HSV thresholds ───────────────────────────────────────────────────────
# Red wraps around H=0/180 in OpenCV HSV, so two ranges are required.
_RED_HSV: List[Tuple[tuple, tuple]] = [
    ((0,   160, 120), (10,  255, 255)),
    ((168, 160, 120), (180, 255, 255)),
]
_GREEN_HSV: List[Tuple[tuple, tuple]] = [
    ((40, 160, 120), (80, 255, 255)),
]

DOT_MIN_AREA_PX: int = 4
DOT_MAX_AREA_PX: int = 1800


# ── Laser dot detector ─────────────────────────────────────────────────────────

def detect_laser_dot(bgr: np.ndarray, color: str) -> Optional[Tuple[float, float]]:
    """Return pixel centroid (cx, cy) of the laser dot, or None."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    ranges = _RED_HSV if color == "red" else _GREEN_HSV

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

    k3 = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k3)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours
             if DOT_MIN_AREA_PX <= cv2.contourArea(c) <= DOT_MAX_AREA_PX]
    if not valid:
        return None
    best = max(valid, key=cv2.contourArea)
    M = cv2.moments(best)
    if M["m00"] == 0:
        return None
    return M["m10"] / M["m00"], M["m01"] / M["m00"]


# ── Homography: pixel → robot mm ───────────────────────────────────────────────

def compute_homography(
    pixel_pts: List[Tuple[float, float]],
    robot_pts: List[Tuple[float, float]],
) -> np.ndarray:
    """
    Compute 3×3 homography H such that  [x,y,1]ᵀ ∝ H @ [u,v,1]ᵀ.

    pixel_pts: list of (u, v) image coordinates
    robot_pts: list of (x_mm, y_mm) robot base-frame coordinates
    Returns H (3×3 float64).
    """
    src = np.array(pixel_pts, dtype=np.float64)
    dst = np.array(robot_pts, dtype=np.float64)
    H, _ = cv2.findHomography(src, dst, method=0)
    if H is None:
        raise RuntimeError("findHomography failed — calibration points may be collinear")
    return H


def apply_homography(H: np.ndarray, u: float, v: float) -> Tuple[float, float]:
    """Map pixel (u, v) → robot (x_mm, y_mm) using homography H."""
    p = H @ np.array([u, v, 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


# ── Main experiment node ───────────────────────────────────────────────────────

class LaserAccuracyExperiment(Node):

    def __init__(self):
        super().__init__("laser_accuracy_experiment")

        self.declare_parameter("laser_detect", True)
        self._use_laser: bool = self.get_parameter("laser_detect").value

        self.bridge = CvBridge()
        self._latest_frame: Optional[np.ndarray] = None
        self._camera_ready: bool = False
        self._H: Optional[np.ndarray] = None   # pixel → robot mm homography

        if self._use_laser:
            self.create_subscription(
                Image, config.COLOR_TOPIC, self._color_cb, 1
            )
            self.get_logger().info(
                f"Subscribed to {config.COLOR_TOPIC} — waiting for first frame…"
            )

    def _color_cb(self, msg: Image) -> None:
        self._latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._camera_ready = True

    # ── Homography calibration ─────────────────────────────────────────────────

    def _grab_dot_pixel(self, timeout_s: float = 5.0) -> Optional[Tuple[float, float]]:
        """
        Block until a laser dot is detected in a fresh frame.
        Returns averaged pixel (u, v) over up to CAPTURE_FRAMES detections.
        """
        readings: List[Tuple[float, float]] = []
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline and len(readings) < CAPTURE_FRAMES:
            rclpy.spin_once(self, timeout_sec=0.04)
            if self._latest_frame is None:
                continue
            uv = detect_laser_dot(self._latest_frame, LASER_COLOR)
            if uv is None:
                continue
            readings.append(uv)

        if not readings:
            return None
        return (
            sum(r[0] for r in readings) / len(readings),
            sum(r[1] for r in readings) / len(readings),
        )

    def _run_homography_calibration(self, motor: DeltaMotorController) -> bool:
        """
        Move to each CALIB_POSITIONS_MM point and record the laser dot pixel.
        Computes and stores the H matrix.  Returns True on success.

        The operator must confirm each capture by pressing Enter.
        This allows them to verify the dot is visible and steady.
        """
        self.get_logger().info(
            "\n" + "=" * 60 +
            "\n  HOMOGRAPHY CALIBRATION"
            "\n  Robot will move to 4 known positions."
            "\n  At each position, press Enter to capture the laser dot."
            "\n" + "=" * 60
        )

        pixel_pts: List[Tuple[float, float]] = []
        robot_pts: List[Tuple[float, float]] = []

        for idx, (x_cal, y_cal) in enumerate(CALIB_POSITIONS_MM):
            self.get_logger().info(
                f"\n  Calibration point {idx + 1}/{len(CALIB_POSITIONS_MM)}: "
                f"moving to ({x_cal:.1f}, {y_cal:.1f}, {EXPERIMENT_Z_MM:.1f}) mm"
            )
            motor.move_xyz(x_cal, y_cal, EXPERIMENT_Z_MM)
            time.sleep(SETTLE_TIME_S + 0.3)

            input(f"  >>> Laser dot visible? Press Enter to capture… ")

            uv = self._grab_dot_pixel(timeout_s=4.0)
            if uv is None:
                self.get_logger().error(
                    f"  Dot not detected at calib point {idx + 1} — check laser and HSV thresholds"
                )
                return False

            self.get_logger().info(
                f"  Captured: pixel=({uv[0]:.1f}, {uv[1]:.1f})  "
                f"robot=({x_cal:.1f}, {y_cal:.1f})"
            )
            pixel_pts.append(uv)
            robot_pts.append((x_cal, y_cal))

        try:
            self._H = compute_homography(pixel_pts, robot_pts)
        except RuntimeError as exc:
            self.get_logger().error(f"Homography failed: {exc}")
            return False

        # Verify reprojection error on calibration points
        max_err = 0.0
        for (u, v), (xr, yr) in zip(pixel_pts, robot_pts):
            xp, yp = apply_homography(self._H, u, v)
            err = math.hypot(xp - xr, yp - yr)
            max_err = max(max_err, err)
        self.get_logger().info(
            f"\n  Calibration done.  Max reprojection error = {max_err:.3f} mm"
        )
        return True

    # ── Laser measurement ──────────────────────────────────────────────────────

    def _capture_dot_mm(self) -> Optional[Tuple[float, float]]:
        """
        Average CAPTURE_FRAMES laser-dot detections and convert to robot mm
        using the calibrated homography.  Returns None if dot not found.
        """
        assert self._H is not None, "Homography not calibrated"
        readings: List[Tuple[float, float]] = []
        deadline = time.monotonic() + CAPTURE_FRAMES * 0.15

        while time.monotonic() < deadline and len(readings) < CAPTURE_FRAMES:
            rclpy.spin_once(self, timeout_sec=0.04)
            if self._latest_frame is None:
                continue
            uv = detect_laser_dot(self._latest_frame, LASER_COLOR)
            if uv is None:
                continue
            readings.append(apply_homography(self._H, uv[0], uv[1]))

        if not readings:
            return None
        return (
            sum(r[0] for r in readings) / len(readings),
            sum(r[1] for r in readings) / len(readings),
        )

    # ── Experiment runner ──────────────────────────────────────────────────────

    def run(self, motor: DeltaMotorController) -> List[Dict]:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(OUTPUT_DIR, f"laser_accuracy_{ts}.csv")
        png_path = os.path.join(OUTPUT_DIR, f"laser_accuracy_{ts}.png")

        # Homography calibration phase
        if self._use_laser:
            if not self._camera_ready:
                self.get_logger().error("No camera frames received — aborting")
                return []
            if not self._run_homography_calibration(motor):
                return []

        results: List[Dict] = []
        n_pts = len(TEST_GRID_MM)

        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  ACCURACY TEST\n"
            f"  {n_pts} positions × {N_REPEATS} repeats — Z = {EXPERIMENT_Z_MM} mm\n"
            f"  Laser: {'ON (' + LASER_COLOR + ')' if self._use_laser else 'OFF (FK only)'}\n"
            f"{'='*60}"
        )

        # Return to home before starting
        motor.move_xyz(0.0, 0.0, EXPERIMENT_Z_MM)
        time.sleep(1.2)

        for pt_idx, (x_cmd, y_cmd) in enumerate(TEST_GRID_MM):
            if not check_workspace(x_cmd, y_cmd, EXPERIMENT_Z_MM):
                self.get_logger().warn(
                    f"Point ({x_cmd:.1f}, {y_cmd:.1f}) out of workspace — skipped"
                )
                continue

            self.get_logger().info(
                f"\n── Point {pt_idx + 1}/{n_pts}  cmd=({x_cmd:.1f}, {y_cmd:.1f}) ──"
            )

            for rep in range(N_REPEATS):
                ok, ik_deg, fb_deg, fk_xyz, fk_err = motor.move_xyz(
                    x_cmd, y_cmd, EXPERIMENT_Z_MM
                )
                time.sleep(SETTLE_TIME_S)

                # Encoder / FK feedback
                x_fk = fk_xyz[0] if fk_xyz else float("nan")
                y_fk = fk_xyz[1] if fk_xyz else float("nan")

                # Laser measurement via homography
                x_laser = y_laser = float("nan")
                if self._use_laser:
                    dot = self._capture_dot_mm()
                    if dot is not None:
                        x_laser, y_laser = dot
                    else:
                        self.get_logger().warn(
                            f"  rep {rep + 1}: laser dot not detected"
                        )

                # Error = command − measured  (paper Eq. 6)
                dx_fk    = x_cmd - x_fk    if not math.isnan(x_fk)    else float("nan")
                dy_fk    = y_cmd - y_fk    if not math.isnan(y_fk)    else float("nan")
                dx_laser = x_cmd - x_laser if not math.isnan(x_laser) else float("nan")
                dy_laser = y_cmd - y_laser if not math.isnan(y_laser) else float("nan")
                dist_fk    = math.hypot(dx_fk,    dy_fk)    if not math.isnan(dx_fk)    else float("nan")
                dist_laser = math.hypot(dx_laser, dy_laser) if not math.isnan(dx_laser) else float("nan")

                log = (
                    f"  rep {rep + 1}: "
                    f"FK=({x_fk:.2f},{y_fk:.2f}) err={dist_fk:.2f} mm"
                )
                if self._use_laser and not math.isnan(x_laser):
                    log += (
                        f"  |  laser=({x_laser:.2f},{y_laser:.2f}) "
                        f"err={dist_laser:.2f} mm"
                    )
                self.get_logger().info(log)

                results.append({
                    "point_idx":  pt_idx,
                    "x_cmd":      x_cmd,
                    "y_cmd":      y_cmd,
                    "repeat":     rep + 1,
                    "x_fk":       round(x_fk,       3),
                    "y_fk":       round(y_fk,       3),
                    "dx_fk":      round(dx_fk,      3),
                    "dy_fk":      round(dy_fk,      3),
                    "dist_fk":    round(dist_fk,    3),
                    "x_laser":    round(x_laser,    3),
                    "y_laser":    round(y_laser,    3),
                    "dx_laser":   round(dx_laser,   3),
                    "dy_laser":   round(dy_laser,   3),
                    "dist_laser": round(dist_laser, 3),
                })

            # Back to home between test points so backlash is always exercised
            # from the same approach direction (matching paper's reset method)
            motor.move_xyz(0.0, 0.0, EXPERIMENT_Z_MM)
            time.sleep(0.5)

        # ── Save CSV ───────────────────────────────────────────────────────────
        fields = [
            "point_idx", "x_cmd", "y_cmd", "repeat",
            "x_fk", "y_fk", "dx_fk", "dy_fk", "dist_fk",
            "x_laser", "y_laser", "dx_laser", "dy_laser", "dist_laser",
        ]
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(results)
        self.get_logger().info(f"\nCSV  → {csv_path}")

        self._print_summary(results)

        try:
            _plot_results(results, png_path, self._use_laser)
            self.get_logger().info(f"Plot → {png_path}")
        except Exception as exc:
            self.get_logger().warn(f"Plot skipped: {exc}")

        return results

    # ── Console summary ────────────────────────────────────────────────────────

    def _print_summary(self, results: List[Dict]) -> None:
        if not results:
            print("No results.")
            return

        print("\n" + "=" * 64)
        print("  ACCURACY / REPEATABILITY SUMMARY")
        print("=" * 64)

        for label, dx_k, dy_k, d_k in [
            ("FK (encoder)",   "dx_fk",    "dy_fk",    "dist_fk"),
            ("Laser (camera)", "dx_laser", "dy_laser", "dist_laser"),
        ]:
            vals = [(r[dx_k], r[dy_k], r[d_k])
                    for r in results if not math.isnan(r[dx_k])]
            if not vals:
                continue
            dxs   = [v[0] for v in vals]
            dys   = [v[1] for v in vals]
            dists = [v[2] for v in vals]
            n  = len(vals)
            mx = sum(dxs) / n
            my = sum(dys) / n
            sx = math.sqrt(sum((v - mx) ** 2 for v in dxs) / max(n - 1, 1))
            sy = math.sqrt(sum((v - my) ** 2 for v in dys) / max(n - 1, 1))

            print(f"\n  [{label}]  n={n}")
            print(f"  X bias            : {mx:+.3f} mm")
            print(f"  Y bias            : {my:+.3f} mm")
            print(f"  X std-dev (σx)    : {sx:.3f} mm")
            print(f"  Y std-dev (σy)    : {sy:.3f} mm")
            print(f"  X repeatability 2σ: ±{2*sx:.3f} mm")
            print(f"  Y repeatability 2σ: ±{2*sy:.3f} mm")
            print(f"  Max dist error    : {max(dists):.3f} mm")

        # Per-point table — use laser column if available, else FK
        use_laser_col = self._use_laser and any(
            not math.isnan(r["dx_laser"]) for r in results
        )
        dx_k = "dx_laser" if use_laser_col else "dx_fk"
        dy_k = "dy_laser" if use_laser_col else "dy_fk"
        src  = "laser" if use_laser_col else "FK"

        by_pt: Dict[Tuple, List] = {}
        for r in results:
            by_pt.setdefault((r["x_cmd"], r["y_cmd"]), []).append(r)

        print(f"\n  Per-point [{src}]")
        print(f"  {'x_cmd':>7} {'y_cmd':>7}  {'n':>3}  "
              f"{'dx bias':>8} {'dy bias':>8}  {'σx':>6} {'σy':>6}  {'max':>7}")
        print("  " + "-" * 62)
        for (xc, yc), recs in sorted(by_pt.items()):
            dxl = [r[dx_k] for r in recs if not math.isnan(r[dx_k])]
            dyl = [r[dy_k] for r in recs if not math.isnan(r[dy_k])]
            if not dxl:
                continue
            n   = len(dxl)
            mxl = sum(dxl) / n
            myl = sum(dyl) / n
            sxl = math.sqrt(sum((v - mxl) ** 2 for v in dxl) / max(n - 1, 1))
            syl = math.sqrt(sum((v - myl) ** 2 for v in dyl) / max(n - 1, 1))
            md  = max(math.hypot(dx, dy) for dx, dy in zip(dxl, dyl))
            print(f"  ({xc:>5.1f},{yc:>5.1f})  {n:>3}  "
                  f"{mxl:>+8.3f} {myl:>+8.3f}  {sxl:>6.3f} {syl:>6.3f}  {md:>7.3f}")
        print("=" * 64)


# ── Matplotlib scatter / ellipse plot ──────────────────────────────────────────

def _plot_results(results: List[Dict], png_path: str, use_laser: bool) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    panels = [("FK (encoder)", "dx_fk", "dy_fk")]
    if use_laser:
        panels.append(("Laser (camera) — homography", "dx_laser", "dy_laser"))

    fig, axes = plt.subplots(1, len(panels), figsize=(7 * len(panels), 6))
    if len(panels) == 1:
        axes = [axes]
    fig.suptitle("Delta Robot Accuracy & Repeatability — Laser Experiment", fontsize=13)

    by_pt: Dict[Tuple, List] = {}
    for r in results:
        by_pt.setdefault((r["x_cmd"], r["y_cmd"]), []).append(r)

    colors = plt.cm.tab10.colors

    for ax, (label, dx_k, dy_k) in zip(axes, panels):
        ax.set_title(label)
        ax.set_xlabel("X error (mm)  [command − measured]")
        ax.set_ylabel("Y error (mm)  [command − measured]")
        ax.axhline(0, color="lightgray", linewidth=0.8)
        ax.axvline(0, color="lightgray", linewidth=0.8)
        ax.set_aspect("equal")
        ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.6)

        patches = []
        for ci, ((xc, yc), recs) in enumerate(sorted(by_pt.items())):
            dxl = [r[dx_k] for r in recs if not math.isnan(r[dx_k])]
            dyl = [r[dy_k] for r in recs if not math.isnan(r[dy_k])]
            if not dxl:
                continue
            color = colors[ci % len(colors)]
            ax.scatter(dxl, dyl, color=color, s=30, zorder=3, alpha=0.85)
            mx = sum(dxl) / len(dxl)
            my = sum(dyl) / len(dyl)
            ax.plot(mx, my, "x", color=color, markersize=9, markeredgewidth=2)
            if len(dxl) > 1:
                # Axis-aligned 2σ ellipse (matches paper Fig. 15)
                sx = 2 * math.sqrt(sum((v - mx) ** 2 for v in dxl) / (len(dxl) - 1))
                sy = 2 * math.sqrt(sum((v - my) ** 2 for v in dyl) / (len(dyl) - 1))
                ax.add_patch(mpatches.Ellipse(
                    (mx, my), width=sx, height=sy,
                    edgecolor=color, facecolor="none",
                    linewidth=1.0, linestyle="--",
                ))
            patches.append(mpatches.Patch(color=color, label=f"({xc:.0f},{yc:.0f})"))

        ax.legend(handles=patches, fontsize=7, loc="upper right",
                  title="cmd (x,y) mm", title_fontsize=7)

    plt.tight_layout()
    plt.savefig(png_path, dpi=150)
    plt.close(fig)


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = LaserAccuracyExperiment()

    if node._use_laser:
        # Wait up to 10 s for the first camera frame
        deadline = time.monotonic() + 10.0
        while not node._camera_ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if not node._camera_ready:
            node.get_logger().error("No camera frame in 10 s — aborting")
            rclpy.shutdown()
            return

    motor = DeltaMotorController()
    motor.connect()

    try:
        node.run(motor)
    finally:
        motor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
