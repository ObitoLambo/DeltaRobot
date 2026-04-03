# =========================
# FILE: fk_ik.py
# Delta Robot FK / IK (Fixed Geometry)
# =========================
import math

# =========================
# FIXED ROBOT GEOMETRY
# =========================
# ⚠️ Keep units consistent (mm or meters)
e  = 35    # end-effector triangle side
f  = 157    # base triangle side
re = 400.0   # lower arm length
rf = 200.0    # upper arm length

# =========================
# Forward Kinematics
# =========================
def delta_calcForward(theta1, theta2, theta3):
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

    w1 = y1*y1 + z1*z1
    w2 = x2*x2 + y2*y2 + z2*z2
    w3 = x3*x3 + y3*y3 + z3*z3

    a1 = (z2 - z1)*(y3 - y1) - (z3 - z1)*(y2 - y1)
    b1 = -((w2 - w1)*(y3 - y1) - (w3 - w1)*(y2 - y1)) / 2.0

    a2 = -(z2 - z1)*x3 + (z3 - z1)*x2
    b2 = ((w2 - w1)*x3 - (w3 - w1)*x2) / 2.0

    a = a1*a1 + a2*a2 + dnm*dnm
    b = 2*(a1*b1 + a2*(b2 - y1*dnm) - z1*dnm*dnm)
    c = (b1*b1 + (b2 - y1*dnm)**2 + dnm*dnm*(z1*z1 - re*re))

    d = b*b - 4*a*c
    if d < 0:
        return -1, 0, 0, 0

    z0 = -0.5*(b + math.sqrt(d)) / a
    x0 = (a1*z0 + b1) / dnm
    y0 = (a2*z0 + b2) / dnm
    return 0, x0, y0, z0


# =========================
# Inverse Kinematics helper
# =========================
def _delta_calcAngleYZ(x0, y0, z0):
    sqrt3 = math.sqrt(3.0)
    tan30 = 1.0 / sqrt3

    y1 = -0.5 * tan30 * f
    y0 -= 0.5 * tan30 * e

    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1) / (2*z0)
    b = (y1 - y0) / z0

    d = rf*(b*b*rf + rf) - (a + b*y1)**2
    if d < 0:
        return -1, 0

    yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1)
    zj = a + b*yj
    theta = math.degrees(math.atan2(-zj, (y1 - yj)))
    if yj > y1:
        theta += 180.0
    return 0, theta


# =========================
# Inverse Kinematics
# =========================
def delta_calcInverse(x0, y0, z0):
    sqrt3 = math.sqrt(3.0)
    sin120 = sqrt3 / 2.0
    cos120 = -0.5

    s, t1 = _delta_calcAngleYZ(x0, y0, z0)
    if s != 0:
        return -1, 0, 0, 0

    s, t2 = _delta_calcAngleYZ(
        x0*cos120 + y0*sin120,
        y0*cos120 - x0*sin120,
        z0
    )
    if s != 0:
        return -1, 0, 0, 0

    s, t3 = _delta_calcAngleYZ(
        x0*cos120 - y0*sin120,
        y0*cos120 + x0*sin120,
        z0
    )
    if s != 0:
        return -1, 0, 0, 0

    return 0, t1, t2, t3


# =========================
# CLI
# =========================
def main():
    print("\n=== DELTA ROBOT FK / IK (FIXED GEOMETRY) ===")
    print(f"e={e}, f={f}, re={re}, rf={rf}")

    while True:
        print("\n1 - IK (x y z -> θ)")
        print("2 - FK (θ -> x y z)")
        print("q - Quit")

        cmd = input(">> ").strip().lower()
        if cmd == 'q':
            break

        if cmd == '1':
            x = float(input("x: "))
            y = float(input("y: "))
            z = float(input("z (negative): "))
            s, t1, t2, t3 = delta_calcInverse(x, y, z)
            if s == 0:
                print(f"θ1={t1:.2f}°, θ2={t2:.2f}°, θ3={t3:.2f}°")
            else:
                print("❌ IK failed")

        elif cmd == '2':
            t1 = float(input("θ1: "))
            t2 = float(input("θ2: "))
            t3 = float(input("θ3: "))
            s, x, y, z = delta_calcForward(t1, t2, t3)
            if s == 0:
                print(f"x={x:.2f}, y={y:.2f}, z={z:.2f}")
            else:
                print("❌ FK failed")


if __name__ == "__main__":
    main()
