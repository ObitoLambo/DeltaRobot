# CHANGES: [2.4] added shared check_workspace helper
# =========================
# FILE 1: fk_ik.py
# (Self-contained Delta IK/FK from the classic tutorial style)
# =========================
import math
e  = 35.0
f  = 157.0
re = 400.0
rf = 200.0
# forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
# returned status: 0=OK, -1=non-existing position
def delta_calcForward(theta1, theta2, theta3, e, f, re, rf):
    sqrt3 = math.sqrt(3.0)
    pi = math.pi
    sin120 = sqrt3 / 2.0
    cos120 = -0.5
    tan60 = sqrt3
    sin30 = 0.5
    tan30 = 1.0 / sqrt3

    t = (f - e) * tan30 / 2.0
    dtr = pi / 180.0

    th1 = theta1 * dtr
    th2 = theta2 * dtr
    th3 = theta3 * dtr

    y1 = -(t + rf * math.cos(th1))
    z1 = -rf * math.sin(th1)

    y2 = (t + rf * math.cos(th2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * math.sin(th2)

    y3 = (t + rf * math.cos(th3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * math.sin(th3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2

    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3

    # x = (a1*z + b1)/dnm
    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

    # y = (a2*z + b2)/dnm
    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

    # a*z^2 + b*z + c = 0
    a = a1 * a1 + a2 * a2 + dnm * dnm
    b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b1 * b1 + (b2 - y1 * dnm) * (b2 - y1 * dnm) + dnm * dnm * (z1 * z1 - re * re))

    d = b * b - 4.0 * a * c
    if d < 0.0:
        return -1, 0.0, 0.0, 0.0

    z0 = -0.5 * (b + math.sqrt(d)) / a
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm
    return 0, x0, y0, z0


def _delta_calcAngleYZ(x0, y0, z0, e, f, re, rf):
    sqrt3 = math.sqrt(3.0)
    pi = math.pi
    tan30 = 1.0 / sqrt3

    y1 = -0.5 * tan30 * f
    y0 -= 0.5 * tan30 * e

    a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
    b = (y1 - y0) / z0

    d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
    if d < 0.0:
        return -1, 0.0

    yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)
    zj = a + b * yj
    theta = math.degrees(math.atan2(-zj, (y1 - yj)))
    if yj > y1:
        theta += 180.0
    return 0, theta


# inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
# returned status: 0=OK, -1=non-existing position
def delta_calcInverse(x0, y0, z0, e, f, re, rf):
    sqrt3 = math.sqrt(3.0)
    sin120 = sqrt3 / 2.0
    cos120 = -0.5

    status, theta1 = _delta_calcAngleYZ(x0, y0, z0, e, f, re, rf)
    if status != 0:
        return -1, 0.0, 0.0, 0.0

    status, theta2 = _delta_calcAngleYZ(
        x0 * cos120 + y0 * sin120,
        y0 * cos120 - x0 * sin120,
        z0, e, f, re, rf
    )
    if status != 0:
        return -1, 0.0, 0.0, 0.0

    status, theta3 = _delta_calcAngleYZ(
        x0 * cos120 - y0 * sin120,
        y0 * cos120 + x0 * sin120,
        z0, e, f, re, rf
    )
    if status != 0:
        return -1, 0.0, 0.0, 0.0

    if theta1 < 0.0 or theta2 < 0.0 or theta3 < 0.0:
        return -1, 0.0, 0.0, 0.0

    return 0, theta1, theta2, theta3


def check_workspace(x_mm: float, y_mm: float, z_mm: float) -> bool:
    """Return True if (x, y, z) in mm is within the configured robot workspace."""
    from delta_common import config

    return (
        abs(x_mm) <= config.X_LIMIT
        and abs(y_mm) <= config.Y_LIMIT
        and config.Z_MIN <= z_mm <= config.Z_MAX
    )
