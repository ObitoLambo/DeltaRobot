#!/usr/bin/env python3
"""
trajectory.py — Cartesian trajectory planning for the delta robot.

Lives in delta_common so it can be imported by any package without
creating circular dependencies.

Generates straight-line Cartesian waypoints between two 3D points using
a symmetric trapezoidal velocity profile (accelerate → cruise → decelerate).

Trapezoidal profile
-------------------

    velocity
      ^
      |   ___________          ← v_max (or v_peak if distance is short)
      |  /           \\
      | /             \\
      |/               \\
      +--+---+-------+--+--→ time
         t_a         T-t_a  T

    * Phase 1  [0,   t_a]:  constant acceleration a_max  → reach v_max
    * Phase 2  [t_a, T-t_a]: cruise at v_max
    * Phase 3  [T-t_a, T]:  constant deceleration a_max  → stop

    If 2·d_accel ≥ distance the profile collapses to a triangle (no cruise
    phase) and the peak speed is limited to sqrt(distance · a_max).

Usage
-----
    from delta_common.trajectory import linear_waypoints

    wps = linear_waypoints(
        p0=(0.0,  0.0, -300.0),
        p1=(50.0, 30.0, -350.0),
        v_max=80.0,    # mm/s  — transport speed
        a_max=200.0,   # mm/s²
        dt=0.05,       # s  → 20 Hz update rate
    )
    controller.execute_trajectory(wps)
"""

import math
from typing import List, Tuple

Point3D = Tuple[float, float, float]


# ── trapezoidal distance profile ─────────────────────────────────────────────

def _trapezoidal_distances(
    distance: float,
    v_max: float,
    a_max: float,
    dt: float,
) -> List[float]:
    """
    Return cumulative arc-length values (mm) sampled at fixed time steps *dt*
    for a point that travels a total of *distance* mm under a trapezoidal
    velocity profile.

    Parameters
    ----------
    distance : total path length in mm (must be > 0)
    v_max    : maximum cruise velocity in mm/s
    a_max    : maximum acceleration / deceleration in mm/s²
    dt       : control-loop period in seconds

    Returns
    -------
    List[float]
        Cumulative distances in mm at each time step.
        The last element equals *distance* exactly.
    """
    if distance < 1e-6:
        return [0.0]

    # Acceleration phase: time and distance to reach v_max
    t_a = v_max / a_max
    d_a = 0.5 * a_max * t_a ** 2

    if 2.0 * d_a >= distance:
        # Triangular profile — distance too short to reach v_max
        t_a = math.sqrt(distance / a_max)
        v_peak = a_max * t_a
        d_a = 0.5 * a_max * t_a ** 2
        T = 2.0 * t_a
    else:
        v_peak = v_max
        t_cruise = (distance - 2.0 * d_a) / v_max
        T = 2.0 * t_a + t_cruise

    samples: List[float] = []
    t = 0.0
    while t <= T + 1e-9:
        if t <= t_a:
            s = 0.5 * a_max * t ** 2
        elif t <= T - t_a:
            s = d_a + v_peak * (t - t_a)
        else:
            dt_rem = T - t
            s = distance - 0.5 * a_max * dt_rem ** 2
        samples.append(min(max(s, 0.0), distance))
        t += dt

    if not samples or samples[-1] < distance - 1e-6:
        samples.append(distance)

    return samples


# ── public API ───────────────────────────────────────────────────────────────

def linear_waypoints(
    p0: Point3D,
    p1: Point3D,
    v_max: float = 80.0,   # mm/s
    a_max: float = 200.0,  # mm/s²
    dt: float = 0.05,      # s  (20 Hz)
) -> List[Point3D]:
    """
    Generate Cartesian waypoints along the straight line from *p0* to *p1*
    using a symmetric trapezoidal velocity profile.

    Parameters
    ----------
    p0, p1  : start / end position as ``(x, y, z)`` in mm (robot base frame)
    v_max   : maximum Cartesian speed in mm/s
    a_max   : maximum Cartesian acceleration in mm/s²
    dt      : time between successive waypoints in seconds

    Returns
    -------
    List[Point3D]
        ``(x, y, z)`` waypoints in mm.
        The last entry is always exactly *p1*.

    Notes
    -----
    * For descend/lift moves use a smaller *v_max* (e.g. 40 mm/s) for
      precision.  Use a larger value (e.g. 100 mm/s) for free-space
      transport moves.
    * Each waypoint must be validated against the robot workspace and joint
      limits before being sent to the motors.  ``execute_trajectory()`` in
      ``DeltaMotorController`` handles this automatically.
    """
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    if distance < 0.5:       # already within 0.5 mm — nothing to do
        return [p1]

    ux, uy, uz = dx / distance, dy / distance, dz / distance

    waypoints: List[Point3D] = []
    for s in _trapezoidal_distances(distance, v_max, a_max, dt):
        waypoints.append((
            p0[0] + s * ux,
            p0[1] + s * uy,
            p0[2] + s * uz,
        ))

    # Overwrite last sample to be exactly p1 (avoids floating-point drift)
    waypoints[-1] = p1
    return waypoints
