#!/usr/bin/env python3
"""
workspace_accuracy_sweep.py — Full X/Y workspace accuracy sweep

Moves the end-effector from X_MIN to X_MAX and Y_MIN to Y_MAX at fixed Z.
At every grid point records the FK encoder position and computes the error.

Outputs
-------
    workspace_sweep_<ts>.csv
    workspace_sweep_<ts>_heatmap_x.png   — X error colour map
    workspace_sweep_<ts>_heatmap_y.png   — Y error colour map
    workspace_sweep_<ts>_vectors.png     — error vector field

Run directly — no ROS needed:
    python3 workspace_accuracy_sweep.py
"""

import csv
import math
import os
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from delta_common import config
from delta_common.fk_ik import check_workspace, delta_calcInverse, e, f, re, rf
from delta_motor_controller.motor_controller import DeltaMotorController

# ── Parameters ─────────────────────────────────────────────────────────────────

EXPERIMENT_Z_MM = -400.0   # fixed Z plane

# X range
X_MIN  = -150.0   # mm
X_MAX  =  150.0   # mm
X_STEP =   25.0   # mm  (smaller = more points, longer run)

# Y range
Y_MIN  = -150.0   # mm
Y_MAX  =  150.0   # mm
Y_STEP =   25.0   # mm

SETTLE_TIME = 1.0  # seconds to wait after each move

# Joint angle safety limit (hardware hard stop at ~56°)
THETA_MAX_DEG = 54.0

VEL_MAX = 1.5   # rad/s — slower for accuracy test
ACC_SET = 3.0   # rad/s²

OUTPUT_DIR = os.path.expanduser("~/delta_ws/experiment_results")


# ── Build grid ─────────────────────────────────────────────────────────────────

def build_grid() -> List[Tuple[float, float]]:
    """
    Generate (x, y) grid points and filter to those that are:
      1. Inside the workspace sphere
      2. Within the THETA_MAX joint limit
    Ordered in a snake pattern (row by row, alternating direction)
    to minimise travel distance.
    """
    xs = []
    x = X_MIN
    while x <= X_MAX + 1e-6:
        xs.append(round(x, 6))
        x += X_STEP

    ys = []
    y = Y_MIN
    while y <= Y_MAX + 1e-6:
        ys.append(round(y, 6))
        y += Y_STEP

    grid = []
    for i, y in enumerate(ys):
        row = xs if i % 2 == 0 else list(reversed(xs))   # snake order
        for x in row:
            if not check_workspace(x, y, EXPERIMENT_Z_MM):
                continue
            st, t1, t2, t3 = delta_calcInverse(x, y, EXPERIMENT_Z_MM, e, f, re, rf)
            if st != 0:
                continue
            # if max(t1, t2, t3) > THETA_MAX_DEG:
            #     continue
            grid.append((x, y))

    return grid


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    grid = build_grid()
    print(f"\n{'='*56}")
    print(f"  Workspace Accuracy Sweep")
    print(f"  X: {X_MIN} → {X_MAX} mm  step={X_STEP} mm")
    print(f"  Y: {Y_MIN} → {Y_MAX} mm  step={Y_STEP} mm")
    print(f"  Z fixed: {EXPERIMENT_Z_MM} mm")
    print(f"  Valid grid points: {len(grid)}")
    print(f"  Estimated time: ~{len(grid)*SETTLE_TIME/60:.1f} min")
    print(f"{'='*56}\n")

    if not grid:
        print("ERROR: no valid grid points — check parameters")
        return

    motor = DeltaMotorController(vel_max=VEL_MAX, acc_set=ACC_SET)
    motor.connect()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(OUTPUT_DIR, f"workspace_sweep_{ts}.csv")
    png_prefix = os.path.join(OUTPUT_DIR, f"workspace_sweep_{ts}")

    results: List[Dict] = []

    # Home first
    motor.move_xyz(0.0, 0.0, EXPERIMENT_Z_MM)
    time.sleep(1.5)

    for idx, (x_cmd, y_cmd) in enumerate(grid):
        ok, ik_deg, fb_deg, fk_xyz, fk_err = motor.move_xyz(
            x_cmd, y_cmd, EXPERIMENT_Z_MM
        )
        time.sleep(SETTLE_TIME)

        x_meas = fk_xyz[0] if fk_xyz else float("nan")
        y_meas = fk_xyz[1] if fk_xyz else float("nan")

        # error = command − measured  (paper Eq. 6)
        err_x = x_cmd - x_meas
        err_y = y_cmd - y_meas
        dist  = math.hypot(err_x, err_y) if not math.isnan(err_x) else float("nan")

        t1f = fb_deg[0] if fb_deg else float("nan")
        t2f = fb_deg[1] if fb_deg else float("nan")
        t3f = fb_deg[2] if fb_deg else float("nan")

        print(f"  [{idx+1:>3}/{len(grid)}]  cmd=({x_cmd:>7.1f},{y_cmd:>7.1f})  "
              f"meas=({x_meas:>7.3f},{y_meas:>7.3f})  "
              f"err=({err_x:>+7.3f},{err_y:>+7.3f})  dist={dist:>6.3f} mm")

        results.append({
            "x_cmd":  x_cmd,   "y_cmd":  y_cmd,
            "x_meas": round(x_meas, 3), "y_meas": round(y_meas, 3),
            "err_x":  round(err_x, 3),  "err_y":  round(err_y, 3),
            "dist":   round(dist,  3),
            "t1_fb":  round(t1f,   4),
            "t2_fb":  round(t2f,   4),
            "t3_fb":  round(t3f,   4),
        })

    motor.shutdown()

    # ── Save CSV ───────────────────────────────────────────────────────────────
    fields = ["x_cmd","y_cmd","x_meas","y_meas","err_x","err_y","dist",
              "t1_fb","t2_fb","t3_fb"]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(results)
    print(f"\nCSV → {csv_path}")

    _print_summary(results)

    try:
        _plot(results, png_prefix)
    except Exception as exc:
        print(f"Plot skipped: {exc}")


# ── Summary ────────────────────────────────────────────────────────────────────

def _mean(v): return sum(v) / len(v)
def _std(v):
    m = _mean(v)
    return math.sqrt(sum((x-m)**2 for x in v) / max(len(v)-1, 1))

def _print_summary(results: List[Dict]):
    valid = [r for r in results if not math.isnan(r["err_x"])]
    if not valid:
        return
    exs = [r["err_x"] for r in valid]
    eys = [r["err_y"] for r in valid]
    ds  = [r["dist"]  for r in valid]

    worst = max(valid, key=lambda r: r["dist"])

    print(f"\n{'='*56}")
    print(f"  WORKSPACE ACCURACY SUMMARY  (n={len(valid)} points)")
    print(f"{'='*56}")
    print(f"  X bias (mean err)  : {_mean(exs):>+8.3f} mm")
    print(f"  Y bias (mean err)  : {_mean(eys):>+8.3f} mm")
    print(f"  X std-dev          : {_std(exs):>8.3f} mm")
    print(f"  Y std-dev          : {_std(eys):>8.3f} mm")
    print(f"  Mean dist error    : {_mean(ds):>8.3f} mm")
    print(f"  Max dist error     : {max(ds):>8.3f} mm")
    print(f"  Worst point        : cmd=({worst['x_cmd']:.1f},{worst['y_cmd']:.1f})")
    print(f"  Best point dist    : {min(ds):>8.3f} mm")
    print(f"{'='*56}")


# ── Plots ──────────────────────────────────────────────────────────────────────

def _plot(results: List[Dict], prefix: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.tri as tri
    import numpy as np

    valid = [r for r in results if not math.isnan(r["err_x"])]
    xs  = np.array([r["x_cmd"]  for r in valid])
    ys  = np.array([r["y_cmd"]  for r in valid])
    exs = np.array([r["err_x"]  for r in valid])
    eys = np.array([r["err_y"]  for r in valid])
    ds  = np.array([r["dist"]   for r in valid])

    # ── Heatmap: X error across workspace ─────────────────────────────────────
    for data, label, fname, cmap in [
        (exs, "X error  (cmd_x − meas_x)  mm", "_heatmap_x.png", "RdBu"),
        (eys, "Y error  (cmd_y − meas_y)  mm", "_heatmap_y.png", "RdBu"),
        (ds,  "Euclidean distance error  mm",   "_heatmap_dist.png", "hot_r"),
    ]:
        fig, ax = plt.subplots(figsize=(7, 6))
        lim = max(abs(data).max(), 0.1)
        vmin = -lim if cmap == "RdBu" else 0
        sc = ax.scatter(xs, ys, c=data, cmap=cmap,
                        vmin=vmin, vmax=lim, s=120, edgecolors="k", linewidths=0.5)
        for xi, yi, di in zip(xs, ys, data):
            ax.annotate(f"{di:+.2f}", (xi, yi),
                        textcoords="offset points", xytext=(0, 6),
                        ha="center", fontsize=6)
        plt.colorbar(sc, ax=ax, label="mm")
        ax.set_xlabel("X commanded (mm)")
        ax.set_ylabel("Y commanded (mm)")
        ax.set_title(f"Workspace Accuracy — {label}\nZ={EXPERIMENT_Z_MM} mm")
        ax.set_aspect("equal")
        ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
        ax.axvline(0, color="gray", linewidth=0.5, linestyle="--")
        ax.grid(True, linestyle=":", alpha=0.4)
        fig.tight_layout()
        p = prefix + fname
        fig.savefig(p, dpi=150)
        plt.close(fig)
        print(f"Plot → {p}")

    # ── Error vector field ─────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_title(f"Error Vector Field  (arrow = command−measured)\nZ={EXPERIMENT_Z_MM} mm",
                 fontsize=11)
    ax.set_xlabel("X commanded (mm)")
    ax.set_ylabel("Y commanded (mm)")
    ax.set_aspect("equal")
    ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax.axvline(0, color="gray", linewidth=0.5, linestyle="--")
    ax.grid(True, linestyle=":", alpha=0.4)

    scale = max(ds.max(), 0.1)
    q = ax.quiver(xs, ys, exs, eys, ds,
                  cmap="hot_r", clim=(0, scale),
                  scale=scale * 6, width=0.004,
                  headwidth=4, headlength=5)
    plt.colorbar(q, ax=ax, label="dist error (mm)")

    # commanded positions as small dots
    ax.scatter(xs, ys, color="navy", s=15, zorder=5, label="commanded pts")
    ax.legend(fontsize=8)

    fig.tight_layout()
    p = prefix + "_vectors.png"
    fig.savefig(p, dpi=150)
    plt.close(fig)
    print(f"Plot → {p}")


if __name__ == "__main__":
    main()
