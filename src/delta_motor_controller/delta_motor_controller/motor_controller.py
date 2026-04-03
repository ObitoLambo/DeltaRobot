#!/usr/bin/env python3

# CHANGES: [2.4] centralized workspace check via delta_common.fk_ik.check_workspace

import math
import time

from delta_common import config
from delta_common.fk_ik import check_workspace, delta_calcForward, delta_calcInverse, e, f, re, rf
from delta_common.trajectory import linear_waypoints  # noqa: F401  (re-exported for convenience)

try:
    from robstride_dynamics import Motor, ParameterType, RobstrideBus
except ImportError:
    from bus import RobstrideBus, Motor
    from protocol import ParameterType


class DeltaMotorController:
    def __init__(self, can_port="can0"):
        self.CAN_PORT = can_port
        self.connected = False
        self._shutdown_done = False

        self.MOTOR_IDS = [1, 2, 3]
        self.MOTOR_NAMES = [f"motor_{mid}" for mid in self.MOTOR_IDS]

        self.VEL_MAX = 1.0
        self.ACC_SET = 2.0

        self.VERIFY_DELAY = 0.30
        self.POS_TOL_MM = 2.0

        motors = {
            name: Motor(id=mid, model="rs-00")
            for name, mid in zip(self.MOTOR_NAMES, self.MOTOR_IDS)
        }
        calibration = {
            name: {"direction": 1, "homing_offset": 0.0}
            for name in self.MOTOR_NAMES
        }

        self.bus = RobstrideBus(self.CAN_PORT, motors, calibration)

    def within_joint_limits(self, t1, t2, t3):
        return (
            config.THETA1_MIN <= t1 <= config.THETA1_MAX
            and config.THETA2_MIN <= t2 <= config.THETA2_MAX
            and config.THETA3_MIN <= t3 <= config.THETA3_MAX
        )

    def connect(self):
        self.bus.connect(handshake=False)
        self.setup_pp_mode()
        self.enable_all()
        self.init_zero()
        self.connected = True
        self._shutdown_done = False

    def setup_pp_mode(self):
        for name in self.MOTOR_NAMES:
            self.bus.write(name, ParameterType.MODE, 1)
            self.bus.write(name, ParameterType.PP_VELOCITY_MAX, float(self.VEL_MAX))
            self.bus.write(name, ParameterType.PP_ACCELERATION_TARGET, float(self.ACC_SET))

    def enable_all(self):
        for name in self.MOTOR_NAMES:
            self.bus.enable(name)
            time.sleep(0.05)

    def init_zero(self):
        for name in self.MOTOR_NAMES:
            self.bus.write(name, ParameterType.POSITION_TARGET, 0.0)
        time.sleep(1.0)

    def move_xyz(self, x, y, z):
        print(f"\nXYZ -> ({x:.2f}, {y:.2f}, {z:.2f})")

        if not check_workspace(x, y, z):
            print(f"Workspace reject: XYZ=({x:.1f},{y:.1f},{z:.1f}) outside limits")
            return False, None, None, None, None

        st_ik, t1, t2, t3 = delta_calcInverse(x, y, z, e, f, re, rf)
        if st_ik != 0:
            print("IK failed")
            return False, None, None, None, None

        ik_deg = (t1, t2, t3)
        if not self.within_joint_limits(t1, t2, t3):
            print(f"Joint limit reject: {ik_deg}")
            return False, ik_deg, None, None, None

        target_radians = [math.radians(theta) for theta in ik_deg]

        for name, rad in zip(self.MOTOR_NAMES, target_radians):
            self.bus.write(name, ParameterType.POSITION_TARGET, float(rad))

        time.sleep(self.VERIFY_DELAY)

        feedback_radians = [
            float(self.bus.read(name, ParameterType.MECHANICAL_POSITION))
            for name in self.MOTOR_NAMES
        ]
        fb_deg = tuple(math.degrees(value) for value in feedback_radians)

        st_fk, x_fk, y_fk, z_fk = delta_calcForward(
            fb_deg[0], fb_deg[1], fb_deg[2], e, f, re, rf
        )
        if st_fk != 0:
            print("FK failed")
            return False, ik_deg, fb_deg, None, None

        fk_xyz = (x_fk, y_fk, z_fk)
        err = math.sqrt((x - x_fk) ** 2 + (y - y_fk) ** 2 + (z - z_fk) ** 2)

        print(f"FK = ({x_fk:.2f}, {y_fk:.2f}, {z_fk:.2f}) | err={err:.2f} mm")

        return err < self.POS_TOL_MM, ik_deg, fb_deg, fk_xyz, err

    def get_current_xyz(self):
        """Read joint positions via CAN feedback and return the FK result.

        Returns
        -------
        (ok, x, y, z) where ok is True if FK succeeded and x/y/z are in mm.
        """
        feedback_radians = [
            float(self.bus.read(name, ParameterType.MECHANICAL_POSITION))
            for name in self.MOTOR_NAMES
        ]
        fb_deg = tuple(math.degrees(v) for v in feedback_radians)
        st, x, y, z = delta_calcForward(fb_deg[0], fb_deg[1], fb_deg[2], e, f, re, rf)
        if st != 0:
            return False, 0.0, 0.0, 0.0
        return True, x, y, z

    def execute_trajectory(self, waypoints, dt=0.05):
        """Move through a list of (x, y, z) Cartesian waypoints.

        Intermediate waypoints are written directly to the motors (IK solved,
        joints commanded) at intervals of *dt* seconds without waiting for the
        full verification delay.  The **final** waypoint goes through the
        standard ``move_xyz()`` with FK verification and tolerance check.

        Parameters
        ----------
        waypoints : list of (x, y, z) tuples in mm (robot base frame)
        dt        : time step between intermediate waypoints in seconds

        Returns
        -------
        bool : True if the final position is within ``POS_TOL_MM``.

        Notes
        -----
        Waypoints that fail the workspace check or have no IK solution are
        silently skipped (the motors keep tracking the previous setpoint).
        Only the final waypoint failure is returned to the caller.
        """
        if not waypoints:
            return True

        # Intermediate waypoints — IK + write only, no verify delay
        for wp in waypoints[:-1]:
            x, y, z = wp
            if not check_workspace(x, y, z):
                continue
            st, t1, t2, t3 = delta_calcInverse(x, y, z, e, f, re, rf)
            if st != 0 or not self.within_joint_limits(t1, t2, t3):
                continue
            radians = [math.radians(t) for t in (t1, t2, t3)]
            for name, rad in zip(self.MOTOR_NAMES, radians):
                self.bus.write(name, ParameterType.POSITION_TARGET, float(rad))
            time.sleep(dt)

        # Final waypoint — full IK + write + FK verification
        ok, *_ = self.move_xyz(*waypoints[-1])
        return ok

    def shutdown(self):
        if self._shutdown_done:
            return

        self._shutdown_done = True

        if not self.connected:
            return

        try:
            for name in self.MOTOR_NAMES:
                self.bus.write(name, ParameterType.POSITION_TARGET, 0.0)

            time.sleep(0.5)

            for name in self.MOTOR_NAMES:
                try:
                    self.bus.disable(name)
                except Exception as ex:
                    print(f"disable failed for {name}: {ex}")
        finally:
            try:
                self.bus.disconnect(disable_torque=True)
            except Exception as ex:
                print(f"bus disconnect failed: {ex}")
            self.connected = False

    def stop(self):
        self.shutdown()
