"""
Workspace error map: loads calibration JSON and returns interpolated
correction vectors to add to a commanded XYZ position.

Single Z-level maps use 2D (XY) interpolation.
Multi-level maps use 3D interpolation.
Points outside the convex hull fall back to nearest-neighbour.
"""
import json
import os
import numpy as np

_cache = {}   # path -> (interps, nears, is_2d, n_points)


def _build(path):
    from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator

    with open(path) as fh:
        data = json.load(fh)

    pts  = np.array([p["cmd_mm"]   for p in data["points"]], dtype=float)
    errs = np.array([p["error_mm"] for p in data["points"]], dtype=float)
    corr = -errs   # correction = -error: shift command in opposite direction

    z_range = float(np.ptp(pts[:, 2]))
    if z_range < 1.0:
        keys   = pts[:, :2]   # 2-D: XY only
        is_2d  = True
    else:
        keys   = pts          # 3-D: XYZ
        is_2d  = False

    interps = tuple(LinearNDInterpolator(keys, corr[:, i]) for i in range(3))
    nears   = tuple(NearestNDInterpolator(keys, corr[:, i]) for i in range(3))
    return interps, nears, is_2d, len(pts)


def _load(path):
    if path in _cache:
        return True
    if not os.path.exists(path):
        print(f"[error_map] file not found: {path}")
        return False
    try:
        interps, nears, is_2d, n = _build(path)
        _cache[path] = (interps, nears, is_2d, n)
        mode = "2-D XY" if is_2d else "3-D XYZ"
        print(f"[error_map] loaded {n} points ({mode}) from {path}")
        return True
    except Exception as exc:
        print(f"[error_map] load failed: {exc}")
        return False


def correction(x: float, y: float, z: float, path: str):
    """Return (cx, cy, cz) correction to ADD to the commanded position."""
    if not _load(path):
        return 0.0, 0.0, 0.0

    interps, nears, is_2d, _ = _cache[path]
    pt = np.array([[x, y]]) if is_2d else np.array([[x, y, z]])

    result = []
    for interp, near in zip(interps, nears):
        v = float(interp(pt)[0])
        if np.isnan(v):
            v = float(near(pt)[0])
        result.append(v)
    return tuple(result)


def point_count(path: str) -> int:
    if not _load(path):
        return 0
    return _cache[path][3]
