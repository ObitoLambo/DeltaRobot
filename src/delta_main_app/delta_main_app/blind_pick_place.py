#!/usr/bin/env python3
"""
blind_pick_place.py — Delta Robot Blind Pick-and-Place Node

Performs pick-and-place using **pre-known** object coordinates — no camera.
Motion uses a trapezoidal velocity profile (delta_common.trajectory) for
smooth, predictable paths.  The gripper is a pneumatic solenoid valve
controlled directly via CAN bus (delta_motor_controller.pneumatic_gripper).

Architecture
------------
    BlindPickAndPlace (ROS 2 Node)
        ├── DeltaMotorController   — 3 × RS-00 motors via CAN (can0, motors 1-3)
        ├── PneumaticGripper       — solenoid valve via CAN (can0, ID 0x04)
        └── PickSequence (thread)  — runs non-blocking pick-and-place FSM

Pick sequence
-------------
    IDLE
      ↓ trigger (service call or /delta/blind_target topic)
    MOVING_TO_PICK   — trapezoidal arc from HOME to (pick_x, pick_y, pick_z)
      ↓
    GRASPING         — pneumatic gripper CLOSE, wait close_settle_s
      ↓
    MOVING_TO_PLACE  — trapezoidal arc to (drop_x, drop_y, drop_z)
      ↓
    DROPPING         — pneumatic gripper OPEN, wait open_settle_s
      ↓
    RESETTING        — trapezoidal arc back to HOME (0, 0, -300 mm)
      ↓
    IDLE

ROS 2 Parameters
----------------
    pick_x / pick_y / pick_z       : pick position in mm (robot base frame)
    drop_x / drop_y / drop_z       : place position in mm
    traj_v_max_mm_s                : max speed in mm/s                   [80.0]
    traj_a_max_mm_s2               : trajectory acceleration in mm/s²   [200.0]
    traj_dt_s                      : trajectory update period in seconds  [0.05]
    pneumatic_can_id               : CAN ID of pneumatic controller      [0x4]
    pneumatic_can_channel          : CAN channel for pneumatic           [can0]

Topics
------
    Subscribed:
        /delta/blind_target   (geometry_msgs/PointStamped)
            Pick at the given (x, y, z) position in mm.

    Published:
        /delta/robot_state    (std_msgs/String)
            Current FSM state name.

Services
--------
    /delta/trigger_pick  (std_srvs/Empty)
        Start a pick cycle at the configured pick_x/y/z parameter position.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Empty

from delta_common import config
from delta_common.fk_ik import check_workspace
from delta_common.trajectory import linear_waypoints
from delta_motor_controller.motor_controller import DeltaMotorController
from delta_motor_controller.pneumatic_gripper import PneumaticGripper

# ── resting / home position ───────────────────────────────────────────────────
HOME_X, HOME_Y, HOME_Z = 0.0, 0.0, -300.0


# ═════════════════════════════════════════════════════════════════════════════
class BlindPickAndPlace(Node):
    """
    ROS 2 node: blind (no-camera) pick-and-place on a delta robot.

    One pick sequence runs at a time in a background daemon thread.
    Additional triggers while busy are logged and dropped.
    """

    def __init__(self):
        super().__init__('blind_pick_place')

        # ── declare parameters ────────────────────────────────────────────
        self.declare_parameter('pick_x',              0.0)
        self.declare_parameter('pick_y',             50.0)
        self.declare_parameter('pick_z',           -350.0)
        self.declare_parameter('drop_x',              0.0)
        self.declare_parameter('drop_y',            -80.0)
        self.declare_parameter('drop_z',           -300.0)
        self.declare_parameter('traj_v_max_mm_s',    80.0)
        self.declare_parameter('traj_a_max_mm_s2',  200.0)
        self.declare_parameter('traj_dt_s',           0.05)
        self.declare_parameter('pneumatic_can_id',   4)

        # ── motor controller ──────────────────────────────────────────────
        self._ctrl = DeltaMotorController(can_port='can0')
        if config.ENABLE_MOTORS:
            self._ctrl.connect()
            self.get_logger().info('Motors connected.')
        else:
            self.get_logger().warn('ENABLE_MOTORS=False — dry-run mode (no CAN output)')

        # ── pneumatic gripper — commands routed through can_driver ────────
        p_id = self.get_parameter('pneumatic_can_id').value
        self._gripper = PneumaticGripper(self, can_id=p_id)
        if config.ENABLE_MOTORS:
            self._gripper.connect()
            self.get_logger().info(
                f'Pneumatic gripper ready  CAN id={p_id}  (via can_driver)')

        # ── publishers ────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/delta/robot_state', 10)

        # ── service ───────────────────────────────────────────────────────
        self._srv = self.create_service(
            Empty, '/delta/trigger_pick', self._trigger_cb)

        # ── subscription ─────────────────────────────────────────────────
        self._sub = self.create_subscription(
            PointStamped, '/delta/blind_target', self._target_cb, 10)

        # ── FSM state ─────────────────────────────────────────────────────
        self._busy = False
        self._lock = threading.Lock()

        self._publish_state('IDLE')
        self.get_logger().info('BlindPickAndPlace ready — waiting for trigger.')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _trigger_cb(self, _req, resp):
        """Service: start pick at the configured pick_x/y/z parameters."""
        p = self.get_parameter
        pick_xyz = (
            p('pick_x').value,
            p('pick_y').value,
            p('pick_z').value,
        )
        self._maybe_start(pick_xyz)
        return resp

    def _target_cb(self, msg: PointStamped) -> None:
        """Topic: pick at the (x, y, z) given in the message (mm, base frame)."""
        pos = (msg.point.x, msg.point.y, msg.point.z)
        if not check_workspace(*pos):
            self.get_logger().warn(
                f'Blind target ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) mm '
                f'is outside workspace — ignored')
            return
        self._maybe_start(pos)

    # ── sequence management ───────────────────────────────────────────────────

    def _maybe_start(self, pick_xyz) -> None:
        with self._lock:
            if self._busy:
                self.get_logger().warn(
                    'Already executing a sequence — new trigger dropped')
                return
            self._busy = True
        threading.Thread(
            target=self._run_sequence, args=(pick_xyz,), daemon=True
        ).start()

    def _run_sequence(self, pick_xyz) -> None:
        """4-waypoint pick-and-place sequence.  Runs in a background thread.

        Waypoints: HOME → PICK (close gripper) → PLACE (open gripper) → HOME
        """
        log = self.get_logger()
        p = self.get_parameter

        place = (p('drop_x').value, p('drop_y').value, p('drop_z').value)
        v_max = p('traj_v_max_mm_s').value
        a_max = p('traj_a_max_mm_s2').value
        dt    = p('traj_dt_s').value
        home  = (HOME_X, HOME_Y, HOME_Z)

        try:
            # ── 1. MOVING_TO_PICK — home → pick position ──────────────────
            self._publish_state('MOVING_TO_PICK')
            log.info(f'[MOVING_TO_PICK]  {home} → {pick_xyz}')
            wps = linear_waypoints(home, pick_xyz, v_max=v_max, a_max=a_max, dt=dt)
            if not self._ctrl.execute_trajectory(wps, dt=dt):
                raise RuntimeError('MOVING_TO_PICK trajectory did not reach target')

            # ── 2. GRASPING — energize pneumatic solenoid ─────────────────
            self._publish_state('GRASPING')
            log.info('[GRASPING]  closing gripper')
            if config.ENABLE_MOTORS:
                self._gripper.close(wait=True)

            # ── 3. MOVING_TO_PLACE — pick position → place position ───────
            self._publish_state('MOVING_TO_PLACE')
            log.info(f'[MOVING_TO_PLACE]  {pick_xyz} → {place}')
            wps = linear_waypoints(pick_xyz, place, v_max=v_max, a_max=a_max, dt=dt)
            if not self._ctrl.execute_trajectory(wps, dt=dt):
                raise RuntimeError('MOVING_TO_PLACE trajectory did not reach target')

            # ── 4. DROPPING — de-energize solenoid ────────────────────────
            self._publish_state('DROPPING')
            log.info('[DROPPING]  opening gripper')
            if config.ENABLE_MOTORS:
                self._gripper.open(wait=True)

        except Exception as exc:
            log.error(f'Sequence aborted: {type(exc).__name__}: {exc}')
            if config.ENABLE_MOTORS:
                self._gripper.open(wait=False)   # safety: release on any failure

        finally:
            # ── 5. RESETTING — return arm to home position ────────────────
            self._publish_state('RESETTING')
            try:
                ok, cx, cy, cz = self._ctrl.get_current_xyz()
                current = (cx, cy, cz) if ok else place
                log.info(f'[RESETTING]  returning home {home}')
                wps = linear_waypoints(current, home, v_max=v_max, a_max=a_max, dt=dt)
                self._ctrl.execute_trajectory(wps, dt=dt)
            except Exception as exc:
                log.error(f'RESETTING failed: {exc}')

            with self._lock:
                self._busy = False
            self._publish_state('IDLE')
            log.info('Sequence complete — IDLE')

    # ── helpers ───────────────────────────────────────────────────────────────

    def _publish_state(self, state: str) -> None:
        m = String()
        m.data = state
        self._state_pub.publish(m)

    def destroy_node(self) -> None:
        if config.ENABLE_MOTORS:
            self._gripper.disconnect()
            self._ctrl.shutdown()
        super().destroy_node()


# ═════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = BlindPickAndPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
