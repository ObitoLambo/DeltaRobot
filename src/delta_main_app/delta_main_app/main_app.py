#!/usr/bin/env python3
"""
main_app.py  -  Delta Robot Main Application
Replaces the stub in delta_main_app/delta_main_app/main_app.py

Subscribes:
    /delta/target_xyz       PointStamped  - object position in base frame (mm)
    /delta/detection_status String        - camera detection state

Publishes:
    /delta/gripper_cmd      Float32       - gripper position [0=open, 1=closed]
    /delta/robot_state      String        - current FSM state name

IMPORTANT - Theta constraint:
    All three motor joint angles must stay within [0 deg, ~85 deg].
    Negative thetas are physically unreachable in this robot's workspace.
    Z_MAX = -196.875 mm is the upper (shallowest) reachable position.
    Approach and lift Z values are clamped to Z_MAX before every move
    so they never generate negative theta solutions.

State Machine:
    IDLE -> CONFIRMING -> APPROACHING -> PICKING ->
    GRASPING -> LIFTING -> TRANSPORTING -> DROPPING -> RESETTING -> IDLE
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray
from std_msgs.msg import Float32, String

from delta_common import config
from delta_common.fk_ik import check_workspace
from delta_motor_controller.motor_controller import DeltaMotorController

# -- Pick-and-place geometry (mm, robot base frame) --------------------
# All Z values must stay within [config.Z_MIN, config.Z_MAX]
# i.e. between -500.0 and -196.875 mm.
# The helpers _approach_z() and _lift_z() clamp automatically.

APPROACH_Z_OFFSET = 25.0
LIFT_Z_OFFSET = 30.0
HOME_X = 0.0
HOME_Y = 0.0
HOME_Z = -350.0

GRASP_WAIT_SEC = 0.4
DROP_WAIT_SEC = 0.4
MOVE_SETTLE_SEC = 0.1

CONFIRM_FRAMES = 2
STABLE_THRESH_MM = 10.0


def _clamp_z(z: float) -> float:
    """
    Clamp Z to the valid workspace range [Z_MIN, Z_MAX].
    This prevents moves that would require negative theta angles.

    Z is negative in this frame (below the base plane):
        Z_MAX = -196.875 mm  (shallowest / closest to base - theta approaches 0 deg)
        Z_MIN = -500.0   mm  (deepest - theta approaches max)

    If an approach/lift offset would push Z above Z_MAX, it is silently
    clamped to Z_MAX so the arm stays reachable.
    """
    return max(config.Z_MIN, min(config.Z_MAX, z))


def _approach_z(z_obj: float) -> float:
    """Z position above the object, clamped to workspace."""
    return _clamp_z(z_obj + APPROACH_Z_OFFSET)


def _lift_z(z_obj: float) -> float:
    """Z position to lift to after grasping, clamped to workspace."""
    return _clamp_z(z_obj + LIFT_Z_OFFSET)


class PickAndPlaceStateMachine:
    """
    Pure state machine - no ROS dependency.
    Receives events from the ROS node and calls motor_controller + gripper.
    All blocking arm moves run in a background thread.
    """

    def __init__(
        self,
        controller: DeltaMotorController,
        gripper_pub,
        state_pub,
        logger,
    ):
        self._ctrl = controller
        self._gripper_pub = gripper_pub
        self._state_pub = state_pub
        self._log = logger
        self._state = "IDLE"

        self._recent_xyz = []
        self._target_xyz = None
        self._busy = False
        self._lock = threading.Lock()

    def detection_update(self, x: float, y: float, z: float) -> None:
        with self._lock:
            if self._busy:
                return

            if self._state == "IDLE":
                self._state = "CONFIRMING"
                self._recent_xyz = []
                self._publish_state()

            if self._state == "CONFIRMING":
                self._recent_xyz.append((x, y, z))
                if len(self._recent_xyz) > CONFIRM_FRAMES:
                    self._recent_xyz.pop(0)

                if len(self._recent_xyz) < CONFIRM_FRAMES:
                    return

                if self._is_stable(self._recent_xyz):
                    avg_x = sum(p[0] for p in self._recent_xyz) / CONFIRM_FRAMES
                    avg_y = sum(p[1] for p in self._recent_xyz) / CONFIRM_FRAMES
                    avg_z = sum(p[2] for p in self._recent_xyz) / CONFIRM_FRAMES
                    self._target_xyz = (avg_x, avg_y, avg_z)
                    self._log.info(
                        f"Target confirmed: ({avg_x:.1f}, {avg_y:.1f}, {avg_z:.1f}) mm"
                        f"  approach_z={_approach_z(avg_z):.1f}"
                        f"  lift_z={_lift_z(avg_z):.1f}"
                    )
                    self._start_sequence()

    def force_target(self, x: float, y: float, z: float) -> bool:
        """Start a pick sequence immediately, bypassing confirmation.
        Camera already confirmed the target — no need to buffer frames.
        Returns False if the robot is currently busy."""
        with self._lock:
            if self._busy:
                return False
            self._target_xyz = (x, y, z)
            self._log.info(f"Queue target: ({x:.1f}, {y:.1f}, {z:.1f}) mm")
            self._start_sequence()
            return True

    def no_detection(self) -> None:
        with self._lock:
            if self._busy:
                return
            if self._state == "CONFIRMING":
                self._recent_xyz = []
                self._state = "IDLE"
                self._publish_state()

    @property
    def state(self) -> str:
        return self._state

    @staticmethod
    def _is_stable(positions) -> bool:
        if len(positions) < 2:
            return False
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]
        spread = math.sqrt(
            (max(xs) - min(xs)) ** 2
            + (max(ys) - min(ys)) ** 2
            + (max(zs) - min(zs)) ** 2
        )
        return spread < STABLE_THRESH_MM

    def _start_sequence(self) -> None:
        self._busy = True
        threading.Thread(target=self._run_sequence, daemon=True).start()

    def _run_sequence(self) -> None:
        from delta_common.fk_ik import delta_calcInverse, e, f, re, rf

        x, y, z = self._target_xyz

        az = _approach_z(z)
        lz = _lift_z(z)

        # At offset XY positions, the shallow approach Z may be outside the
        # reachable workspace (delta workspace shrinks near the ceiling).
        # If approach has no IK solution, skip it and go straight to pick depth.
        st_az, *_ = delta_calcInverse(x, y, az, e, f, re, rf)
        use_approach = (st_az == 0)
        if not use_approach:
            self._log.warn(
                f"Approach Z={az:.1f} unreachable at XY=({x:.1f},{y:.1f}) "
                f"— skipping approach, descending directly to Z={z:.1f}"
            )

        if lz != z + LIFT_Z_OFFSET:
            self._log.warn(
                f"Lift Z clamped: {z + LIFT_Z_OFFSET:.1f} -> {lz:.1f} mm "
                f"(Z_MAX={config.Z_MAX:.1f})"
            )

        try:
            self._set_state("APPROACHING")
            if use_approach:
                if not self._move(x, y, az):
                    raise RuntimeError("APPROACH failed")
                time.sleep(MOVE_SETTLE_SEC)

            self._set_state("PICKING")
            if not self._move(x, y, z):
                raise RuntimeError("PICK descend failed")
            time.sleep(MOVE_SETTLE_SEC)

            self._set_state("GRASPING")
            self._gripper(1.0)
            time.sleep(GRASP_WAIT_SEC)

            self._set_state("LIFTING")
            if not self._move(x, y, lz):
                raise RuntimeError("LIFT failed")
            time.sleep(MOVE_SETTLE_SEC)

            self._set_state("TRANSPORTING")
            if not self._move(config.PLACE_X, config.PLACE_Y, config.PLACE_Z):
                raise RuntimeError("TRANSPORT failed")
            time.sleep(MOVE_SETTLE_SEC)

            self._set_state("DROPPING")
            self._gripper(0.0)
            time.sleep(DROP_WAIT_SEC)

        except RuntimeError as exc:
            self._log.error(f"Sequence aborted: {exc}")
            self._gripper(0.0)

        finally:
            self._set_state("RESETTING")
            self._move(HOME_X, HOME_Y, HOME_Z, raw=True)

            with self._lock:
                self._state = "IDLE"
                self._busy = False
                self._target_xyz = None
                self._recent_xyz = []
            self._publish_state()
            self._log.info("Sequence complete - IDLE")

    def _move(self, x: float, y: float, z: float, raw: bool = False) -> bool:
        from delta_common.fk_ik import delta_calcInverse, e, f, re, rf

        st, t1, t2, t3 = delta_calcInverse(x, y, z, e, f, re, rf)
        if st != 0:
            self._log.error(f"IK no solution for ({x:.1f}, {y:.1f}, {z:.1f})")
            return False
        if t1 < 0 or t2 < 0 or t3 < 0:
            self._log.error(
                f"Negative theta rejected: theta=({t1:.1f}, {t2:.1f}, {t3:.1f}) "
                f"for ({x:.1f}, {y:.1f}, {z:.1f}) - Z too shallow?"
            )
            return False

        self._log.info(
            f"  Move ({x:.1f}, {y:.1f}, {z:.1f})  "
            f"theta=({t1:.1f}, {t2:.1f}, {t3:.1f})"
        )
        ok, ik_deg, fb_deg, fk_xyz, err = self._ctrl.move_xyz(x, y, z, raw=raw)
        if ok:
            self._log.info(f"  FK err={err:.2f} mm  OK")
        else:
            self._log.warn("  Move FAILED")
        return ok

    def _gripper(self, pos: float) -> None:
        self._log.info(f'  Gripper -> {"CLOSE" if pos > 0.5 else "OPEN"}')
        self._gripper_pub(pos)

    def _set_state(self, state: str) -> None:
        self._state = state
        self._publish_state()
        self._log.info(f"[FSM] -> {state}")

    def _publish_state(self) -> None:
        self._state_pub(self._state)


class DeltaMainApp(Node):
    def __init__(self):
        super().__init__("delta_main_app")

        self._ctrl = DeltaMotorController(can_port="can0")
        if config.ENABLE_MOTORS:
            self._ctrl.connect()
            self.get_logger().info("Motors connected.")
        else:
            self.get_logger().warn("ENABLE_MOTORS=False - dry run mode")

        self._gripper_pub = self.create_publisher(Float32, "/delta/gripper_cmd", 10)
        self._state_pub = self.create_publisher(String, "/delta/robot_state", 10)

        self._fsm = PickAndPlaceStateMachine(
            controller=self._ctrl,
            gripper_pub=self._send_gripper,
            state_pub=self._send_state,
            logger=self.get_logger(),
        )

        self._target_queue: list = []
        self._belt_vy_mm_s: float = 0.0
        self._queue_lock = threading.Lock()

        self._sub_all_targets = self.create_subscription(
            PoseArray, "/delta/all_targets", self._all_targets_callback, 10
        )
        self._sub_velocity = self.create_subscription(
            PointStamped, "/delta/object_velocity_mm_s", self._velocity_callback, 10
        )
        self._sub_status = self.create_subscription(
            String, "/delta/detection_status", self._status_callback, 10
        )
        self.create_timer(0.1, self._queue_timer)

        self.get_logger().info(
            f"DeltaMainApp ready  "
            f"Z_workspace=[{config.Z_MIN:.0f}, {config.Z_MAX:.0f}] mm  "
            f"place=({config.PLACE_X:.0f}, {config.PLACE_Y:.0f}, {config.PLACE_Z:.0f}) mm"
        )

    def _all_targets_callback(self, msg: PoseArray) -> None:
        with self._queue_lock:
            if self._fsm.state != "IDLE":
                return
            if self._target_queue:
                return
            candidates = [
                (p.position.x * 1000.0, p.position.y * 1000.0, p.position.z * 1000.0)
                for p in msg.poses
            ]
            if not candidates:
                return
            candidates.sort(key=lambda p: math.sqrt(p[0] ** 2 + p[1] ** 2))
            self._target_queue = candidates
            self.get_logger().info(f"Queued {len(candidates)} target(s)")

    def _velocity_callback(self, msg: PointStamped) -> None:
        self._belt_vy_mm_s = msg.point.y

    def _queue_timer(self) -> None:
        with self._queue_lock:
            if self._fsm.state != "IDLE" or not self._target_queue:
                return
            x, y, z = self._target_queue.pop(0)
            dist_mm = math.sqrt(x ** 2 + y ** 2)
            t_travel = dist_mm / max(config.ROBOT_SPEED_MM_S, 1.0)
            y_pred = y + self._belt_vy_mm_s * t_travel
            if not check_workspace(x, y_pred, z):
                self.get_logger().warn(
                    f"Predicted pick ({x:.1f}, {y_pred:.1f}, {z:.1f}) outside workspace — skipped"
                )
                return
            self._fsm.force_target(x, y_pred, z)

    def _status_callback(self, msg: String) -> None:
        if "NO_DETECTION" in msg.data or "LOST" in msg.data:
            self._fsm.no_detection()

    def _send_gripper(self, pos: float) -> None:
        m = Float32()
        m.data = float(pos)
        self._gripper_pub.publish(m)

    def _send_state(self, state: str) -> None:
        m = String()
        m.data = state
        self._state_pub.publish(m)

    def destroy_node(self) -> None:
        self._send_gripper(0.0)
        if config.ENABLE_MOTORS:
            self._ctrl.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeltaMainApp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
