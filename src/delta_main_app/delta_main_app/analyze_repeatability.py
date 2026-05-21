#!/usr/bin/env python3
"""
analyze_repeatability.py — Repeatability & Accuracy Analysis

Reads the CSV produced by laser_accuracy_experiment.py and generates
three graphs used to characterize robot positioning performance:

  Graph 1 — Absolute position scatter (robot XY space)
      Commanded position (★) + all repeated laser measurements (●)
      with 2σ ellipse around each cluster.
      Shows BOTH accuracy (how far from commanded) and
      repeatability (how spread the cluster is).
      → Matches paper Fig. 15.

  Graph 2 — Error vector map (robot XY space)
      Arrows from commanded (★) to mean measured position (●).
      Length/direction of each arrow = systematic bias error.
      → Matches paper Fig. 13.

  Graph 3 — Per-position X/Y error bar chart
      Grouped bars: X error and Y error for each test position.
      Error bars show ±1σ (std dev across repeats).
      → Tabular equivalent of paper Table III.

Usage
-----
    python3 analyze_repeatability.py <path/to/laser_accuracy_YYYYMMDD_HHMMSS.csv>

    # or just point at the results directory to load the latest file:
    python3 analyze_repeatability.py ~/delta_ws/experiment_results/
"""

import csv
import math
import os
import sys
from typing import Dict, List, Tuple


# ── CSV loader ─────────────────────────────────────────────────────────────────

def load_csv(path: str) -> List[Dict]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = []
        for row in reader:
            for k in row:
                try:
                    row[k] = float(row[k])
                except ValueError:
                    pass
            rows.append(row)
    return rows


def find_latest_csv(directory: str) -> str:
    files = [
        os.path.join(directory, f)
        for f in os.listdir(directory)
        if f.startswith("laser_accuracy_") and f.endswith(".csv")
    ]
    if not files:
        raise FileNotFoundError(f"No laser_accuracy_*.csv files in {directory}")
    return max(files, key=os.path.getmtime)


# ── Statistics helpers ─────────────────────────────────────────────────────────

def mean(vals: List[float]) -> float:
    return sum(vals) / len(vals)


def std(vals: List[float]) -> float:
    m = mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / max(len(vals) - 1, 1))


def per_point_stats(
    results: List[Dict],
    x_key: str,
    y_key: str,
) -> Dict[Tuple[float, float], Dict]:
    """
    Group results by (x_cmd, y_cmd) and compute per-point statistics.
    Returns dict keyed by (x_cmd, y_cmd) with fields:
        xs, ys          — raw measured positions
        mx, my          — mean measured position
        sx, sy          — std dev of measured position
        bias_x, bias_y  — command − mean_measured  (paper Eq. 6)
        max_err         — max Euclidean distance from commanded
    """
    by_pt: Dict[Tuple, List] = {}
    for r in results:
        k = (r["x_cmd"], r["y_cmd"])
        by_pt.setdefault(k, []).append(r)

    stats = {}
    for (xc, yc), recs in by_pt.items():
        xs = [r[x_key] for r in recs if not math.isnan(r[x_key])]
        ys = [r[y_key] for r in recs if not math.isnan(r[y_key])]
        if not xs:
            continue
        mx_ = mean(xs)
        my_ = mean(ys)
        stats[(xc, yc)] = {
            "xs": xs, "ys": ys,
            "mx": mx_, "my": my_,
            "sx": std(xs), "sy": std(ys),
            "bias_x": xc - mx_,
            "bias_y": yc - my_,
            "max_err": max(
                math.hypot(x - xc, y - yc) for x, y in zip(xs, ys)
            ),
        }
    return stats


# ── The three plots ────────────────────────────────────────────────────────────

def plot_all(results: List[Dict], out_prefix: str, use_laser: bool) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import numpy as np

    # Choose laser or FK column
    if use_laser and any(not math.isnan(r["x_laser"]) for r in results):
        x_k, y_k = "x_laser", "y_laser"
        dx_k, dy_k = "dx_laser", "dy_laser"
        src = "Laser (homography)"
    else:
        x_k, y_k = "x_fk", "y_fk"
        dx_k, dy_k = "dx_fk", "dy_fk"
        src = "FK (encoder)"

    stats = per_point_stats(results, x_k, y_k)
    colors = plt.cm.tab10.colors

    # ── Graph 1: Absolute position scatter + 2σ ellipses ──────────────────────
    fig1, ax1 = plt.subplots(figsize=(8, 8))
    ax1.set_title(
        f"Graph 1 — Position Repeatability [{src}]\n"
        "★ = commanded · ● = measured · ellipse = 2σ",
        fontsize=11,
    )
    ax1.set_xlabel("X position (mm)")
    ax1.set_ylabel("Y position (mm)")
    ax1.set_aspect("equal")
    ax1.grid(True, linestyle="--", linewidth=0.4, alpha=0.6)

    for ci, ((xc, yc), s) in enumerate(sorted(stats.items())):
        color = colors[ci % len(colors)]
        # Commanded position
        ax1.plot(xc, yc, "*", color=color, markersize=14, zorder=5)
        # Repeated measurements
        ax1.scatter(s["xs"], s["ys"], color=color, s=25, alpha=0.8, zorder=4)
        # 2σ ellipse centred on mean measured position
        if len(s["xs"]) > 1:
            ell = mpatches.Ellipse(
                (s["mx"], s["my"]),
                width=2 * s["sx"],   # semi-axis = σ, full width = 2σ
                height=2 * s["sy"],
                edgecolor=color, facecolor="none",
                linewidth=1.2, linestyle="--",
            )
            ax1.add_patch(ell)
        # Label
        ax1.annotate(
            f"({xc:.0f},{yc:.0f})",
            xy=(xc, yc), xytext=(4, 4), textcoords="offset points",
            fontsize=7, color=color,
        )

    fig1.tight_layout()
    p1 = out_prefix + "_graph1_scatter.png"
    fig1.savefig(p1, dpi=150)
    plt.close(fig1)
    print(f"  Graph 1 → {p1}")

    # ── Graph 2: Error vector map ──────────────────────────────────────────────
    fig2, ax2 = plt.subplots(figsize=(8, 8))
    ax2.set_title(
        f"Graph 2 — Error Vector Map [{src}]\n"
        "Arrow: commanded (★) → mean measured (●)",
        fontsize=11,
    )
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Y (mm)")
    ax2.set_aspect("equal")
    ax2.grid(True, linestyle="--", linewidth=0.4, alpha=0.6)
    ax2.axhline(0, color="gray", linewidth=0.6)
    ax2.axvline(0, color="gray", linewidth=0.6)

    max_bias = max(
        math.hypot(s["bias_x"], s["bias_y"]) for s in stats.values()
    ) or 1.0

    for ci, ((xc, yc), s) in enumerate(sorted(stats.items())):
        color = colors[ci % len(colors)]
        ax2.plot(xc, yc, "*", color=color, markersize=12, zorder=5)
        ax2.plot(s["mx"], s["my"], "o", color=color, markersize=7, zorder=4)
        ax2.annotate(
            "",
            xy=(s["mx"], s["my"]),
            xytext=(xc, yc),
            arrowprops=dict(
                arrowstyle="->",
                color=color,
                lw=1.5,
            ),
        )
        # Error magnitude label near arrow midpoint
        mx_mid = (xc + s["mx"]) / 2
        my_mid = (yc + s["my"]) / 2
        err = math.hypot(s["bias_x"], s["bias_y"])
        ax2.annotate(
            f"{err:.2f}mm",
            xy=(mx_mid, my_mid),
            fontsize=7, color=color,
            ha="center",
        )

    fig2.tight_layout()
    p2 = out_prefix + "_graph2_vectors.png"
    fig2.savefig(p2, dpi=150)
    plt.close(fig2)
    print(f"  Graph 2 → {p2}")

    # ── Graph 3: Per-position X/Y bias bar chart with ±1σ error bars ──────────
    positions = sorted(stats.keys())
    labels = [f"({xc:.0f},{yc:.0f})" for xc, yc in positions]
    bias_x = [stats[p]["bias_x"] for p in positions]
    bias_y = [stats[p]["bias_y"] for p in positions]
    err_x  = [stats[p]["sx"]     for p in positions]
    err_y  = [stats[p]["sy"]     for p in positions]

    x_idx = np.arange(len(positions))
    bar_w = 0.35

    fig3, ax3 = plt.subplots(figsize=(max(8, len(positions) * 1.1), 5))
    ax3.set_title(
        f"Graph 3 — Per-Position Bias ± 1σ [{src}]\n"
        "(error = command − measured; ±1σ error bars across repeats)",
        fontsize=11,
    )
    ax3.bar(x_idx - bar_w / 2, bias_x, bar_w, yerr=err_x,
            label="X bias", color="#2196F3", alpha=0.8,
            capsize=4, error_kw={"elinewidth": 1.2})
    ax3.bar(x_idx + bar_w / 2, bias_y, bar_w, yerr=err_y,
            label="Y bias", color="#F44336", alpha=0.8,
            capsize=4, error_kw={"elinewidth": 1.2})
    ax3.axhline(0, color="black", linewidth=0.8)
    ax3.set_xticks(x_idx)
    ax3.set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
    ax3.set_ylabel("Position error (mm)")
    ax3.set_xlabel("Commanded position (x,y) mm")
    ax3.legend()
    ax3.grid(True, axis="y", linestyle="--", linewidth=0.4, alpha=0.6)

    fig3.tight_layout()
    p3 = out_prefix + "_graph3_bars.png"
    fig3.savefig(p3, dpi=150)
    plt.close(fig3)
    print(f"  Graph 3 → {p3}")

    # ── Text summary ───────────────────────────────────────────────────────────
    all_xs = [x for s in stats.values() for x in s["xs"]]
    all_ys = [y for s in stats.values() for y in s["ys"]]
    all_xc = [xc for (xc, yc) in stats for _ in stats[(xc, yc)]["xs"]]
    all_yc = [yc for (xc, yc) in stats for _ in stats[(xc, yc)]["ys"]]
    all_dx = [xc - x for xc, x in zip(all_xc, all_xs)]
    all_dy = [yc - y for yc, y in zip(all_yc, all_ys)]

    mx_global = mean(all_dx)
    my_global = mean(all_dy)
    sx_global = std(all_dx)
    sy_global = std(all_dy)

    print("\n  ── Overall statistics ──────────────────────────────────")
    print(f"  Source             : {src}")
    print(f"  Total measurements : {len(all_dx)}")
    print(f"  X bias (mean err)  : {mx_global:+.3f} mm")
    print(f"  Y bias (mean err)  : {my_global:+.3f} mm")
    print(f"  X repeatability 2σ : ±{2*sx_global:.3f} mm")
    print(f"  Y repeatability 2σ : ±{2*sy_global:.3f} mm")
    print(f"  X std-dev σx       : {sx_global:.3f} mm")
    print(f"  Y std-dev σy       : {sy_global:.3f} mm")
    print(f"  Max error          : {max(math.hypot(dx,dy) for dx,dy in zip(all_dx,all_dy)):.3f} mm")
    print("  ────────────────────────────────────────────────────────")

    print("\n  Per-point:")
    print(f"  {'cmd (x,y)':>14}  {'bias_x':>7} {'bias_y':>7}  {'σx':>6} {'σy':>6}  {'max_err':>7}  {'n':>3}")
    print("  " + "-" * 60)
    for (xc, yc) in positions:
        s = stats[(xc, yc)]
        print(
            f"  ({xc:>5.0f},{yc:>5.0f})  "
            f"{s['bias_x']:>+7.3f} {s['bias_y']:>+7.3f}  "
            f"{s['sx']:>6.3f} {s['sy']:>6.3f}  "
            f"{s['max_err']:>7.3f}  {len(s['xs']):>3}"
        )


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        default_dir = os.path.expanduser("~/delta_ws/experiment_results")
        print(f"No path given — using latest CSV in {default_dir}")
        path = find_latest_csv(default_dir)
    else:
        path = sys.argv[1]
        if os.path.isdir(path):
            path = find_latest_csv(path)

    if not os.path.isfile(path):
        print(f"File not found: {path}")
        sys.exit(1)

    print(f"Loading: {path}")
    results = load_csv(path)
    print(f"  {len(results)} rows loaded")

    use_laser = any(not math.isnan(r.get("x_laser", float("nan"))) for r in results)

    out_prefix = path.replace(".csv", "")
    print("\nGenerating plots…")
    plot_all(results, out_prefix, use_laser)
    print("\nDone.")


if __name__ == "__main__":
    main()
