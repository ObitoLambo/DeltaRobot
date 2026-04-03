# =========================
# FILE 3: Delta_description.py
# This is the MAIN script you run.
# It computes IK ONCE, then commands all 3 motors using ONE shared CAN loop.
# =========================
import time
from robstride_dynamics import delta_calcForward, delta_calcInverse
from position_control_shared import SharedCanTransport, DeltaMotorGroupController, MotorCalib

# -------------------------
# Delta geometry (mm)
# -------------------------
e = 35.0     # end effector triangle side
f = 157.0    # base triangle side
re = 400.0   # parallelogram (lower arm)
rf = 200.0   # upper arm

def main():
    # Example target in mm
    target_x = 100.0
    target_y = 150.0
    target_z = -300.0

    # 1) IK once
    status, t1, t2, t3 = delta_calcInverse(target_x, target_y, target_z, e, f, re, rf)
    if status != 0:
        print("❌ IK failed: invalid XYZ")
        return

    print(f"IK result: theta1={t1:.2f} deg, theta2={t2:.2f} deg, theta3={t3:.2f} deg")

    # 2) FK check once (math validation only)
    fk_status, x0, y0, z0 = delta_calcForward(t1, t2, t3, e, f, re, rf)
    if fk_status == 0:
        print(f"FK check: x0={x0:.2f}, y0={y0:.2f}, z0={z0:.2f}")
    else:
        print("⚠️ FK failed (should not happen if IK was OK)")

    # 3) ONE shared CAN transport + ONE loop
    calib = {
        1: MotorCalib(direction=1, homing_offset_deg=0.0),
        2: MotorCalib(direction=1, homing_offset_deg=0.0),
        3: MotorCalib(direction=1, homing_offset_deg=0.0),
    }

    transport = SharedCanTransport(channel="can0")
    group = DeltaMotorGroupController(transport=transport, calib=calib, hz=50.0)

    try:
        group.start()

        # Set targets (all 3 together)
        group.set_targets_deg(t1, t2, t3)
        print(f"🎯 Targets set: {t1:.2f}, {t2:.2f}, {t3:.2f} deg")

        # hold for some seconds
        time.sleep(3.0)

        # Optional: return to zero
        group.set_targets_deg(0.0, 0.0, 0.0)
        print("🏠 Returning to 0 deg...")
        time.sleep(3.0)

    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C")
    finally:
        group.stop()
        print("✅ Stopped cleanly.")


if __name__ == "__main__":
    main()
