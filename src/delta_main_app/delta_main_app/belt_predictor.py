#!/usr/bin/env python3
"""
belt_predictor.py — Belt velocity estimator and pick-point predictor.

Maintains a rolling window of (timestamp, y_mm) samples from the camera,
fits a linear regression to estimate belt velocity, then predicts where a
detected object will be when the robot end-effector arrives.

Pick-point timing model (medium preset, triangular profile):
    t_travel  = 2 * sqrt(dist / TRAJ_A_MAX_MM_S2)
    t_descend = 2 * sqrt(DESCEND_DIST_MM / TRAJ_A_MAX_MM_S2)
    t_total   = LATENCY_S + t_travel + t_descend
    y_pick    = y_detected + vy * t_total
"""

import math
from dataclasses import dataclass
from typing import Optional

from delta_common import config

_LATENCY_S       = 0.083   # camera capture + ROS comms latency
_DESCEND_DIST_MM = 60.0    # approach Z to belt surface (Z=-350 → Z=-410)
_WINDOW_S        = 2.0     # rolling regression window
_MIN_SAMPLES     = 5       # minimum samples before velocity is trusted


@dataclass
class PredictResult:
    y_pick:      float
    valid:       bool
    t_total:     float
    belt_offset: float
    vy_mm_s:     float


class BeltPredictor:
    """Estimates belt velocity and predicts object Y at pick time."""

    def __init__(self):
        self._samples: list = []   # list of (timestamp_s, y_mm)

    # ── data ingestion ────────────────────────────────────────────────────────

    def update_velocity(self, y_mm: float, timestamp: float) -> None:
        """Record a new Y observation. Call on every detection frame."""
        # If Y jumped beyond what belt physics allow, the tracked object changed
        # (previous one was picked). Stale samples from the old object would corrupt
        # the velocity regression — discard them before adding the new reading.
        if self._samples:
            last_y = self._samples[-1][1]
            dt = max(timestamp - self._samples[-1][0], 0.0)
            # Belt only moves in -Y.  Clear window if Y jumped in the wrong
            # direction (+Y) or moved more than physically possible in -Y.
            # 5 mm tolerance absorbs camera noise on the same object.
            if (y_mm > last_y + 5.0 or
                    y_mm < last_y - config.CONVEYOR_VY_MAX_MM_S * dt - 5.0):
                self._samples.clear()
        self._samples.append((timestamp, y_mm))
        cutoff = timestamp - _WINDOW_S
        self._samples = [(t, y) for t, y in self._samples if t >= cutoff]

    def clear(self) -> None:
        self._samples.clear()

    # ── velocity estimate ─────────────────────────────────────────────────────

    @property
    def measured_vy(self) -> Optional[float]:
        """Current belt velocity estimate in mm/s (negative = toward EXIT).
        Returns None if insufficient data."""
        return self._estimate_vy()

    def _estimate_vy(self) -> Optional[float]:
        if len(self._samples) < _MIN_SAMPLES:
            return None

        times = [s[0] for s in self._samples]
        ys    = [s[1] for s in self._samples]
        t_mean = sum(times) / len(times)
        y_mean = sum(ys)    / len(ys)

        num   = sum((t - t_mean) * (y - y_mean) for t, y in zip(times, ys))
        denom = sum((t - t_mean) ** 2            for t in times)

        if denom < 1e-9:
            return None

        vy = num / denom

        # Sanity bounds from config
        if abs(vy) < config.CONVEYOR_VY_MIN_MM_S:
            return 0.0
        if abs(vy) > config.CONVEYOR_VY_MAX_MM_S:
            vy = math.copysign(config.CONVEYOR_VY_MAX_MM_S, vy)

        return vy

    # ── pick-point prediction ─────────────────────────────────────────────────

    def predict(self, x_mm: float, y_mm: float, current_robot_y: float = 0.0) -> PredictResult:
        """
        Predict where the object will be when the robot EEF arrives.

        Iterates 4 times: y_pick depends on t_travel, which depends on
        dist to y_pick. Converges in 3-4 iterations.

        Returns PredictResult with valid=False if velocity is unknown or the
        predicted pick point is outside the workspace.
        """
        vy = self._estimate_vy()
        if vy is None:
            vy = -config.CONVEYOR_BELT_SPEED_MM_S

        a = max(config.TRAJ_A_MAX_MM_S2, 1.0)
        t_descend = 2.0 * math.sqrt(_DESCEND_DIST_MM / a)

        y_pred = y_mm
        t_travel = 0.0
        for _ in range(4):
            dist_mm  = abs(y_pred - current_robot_y)
            t_travel = 2.0 * math.sqrt(dist_mm / a)
            t_total  = _LATENCY_S + t_travel + t_descend
            y_pred   = y_mm + vy * t_total

        belt_offset = y_pred - y_mm
        valid = (
            abs(x_mm)  <= config.X_LIMIT
            and abs(y_pred) <= config.Y_LIMIT
        )

        return PredictResult(
            y_pick=y_pred,
            valid=valid,
            t_total=t_total,
            belt_offset=belt_offset,
            vy_mm_s=vy,
        )
