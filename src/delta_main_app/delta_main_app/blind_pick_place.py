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

import random
import threading
import time

import math

import rclpy
import rclpy.parameter
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray
from std_msgs.msg import String
from std_srvs.srv import Empty

from delta_common import config
from delta_common.fk_ik import check_workspace
from delta_common.trajectory import linear_waypoints
from delta_motor_controller.motor_controller import DeltaMotorController
from delta_motor_controller.pneumatic_gripper import PneumaticGripper

# ── resting / home position ───────────────────────────────────────────────────
HOME_X, HOME_Y, HOME_Z = 0.0, 0.0, -350.0

# ── speed presets ─────────────────────────────────────────────────────────────
# Keys: (traj_v_max_mm_s, traj_a_max_mm_s2, motor_vel_rad_s, motor_acc_rad_s2)
# Motor limits are sized so omega ≈ v_mm_s / rf (rf = 200 mm) with ~2× margin.
# Columns: (traj_v_max_mm_s, traj_a_max_mm_s2, motor_vel_rad_s, motor_acc_rad_s2, verify_delay_s)
# verify_delay_s — time to wait after the final waypoint for the motor to settle.
# Higher ACC means the motor brakes harder and settles faster → shorter delay is safe.
# Columns: (traj_v_mm_s, traj_a_mm_s2, motor_vel_rad_s, motor_acc_rad_s2, verify_timeout_s)
# verify_timeout_s is now a MAXIMUM wait — move_xyz exits as soon as the motor
# settles within POS_TOL_MM, so making this generous costs nothing for fast moves.
SPEED_PRESETS = {
    'low':    (  80.0,   200.0,  1.0,   2.0, 0.50),
    'medium': (2000.0,  5000.0, 25.0,  50.0, 0.50),
    'max':    (4000.0, 10000.0, 50.0, 100.0, 0.50),
}


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
        self.declare_parameter('pick_y',             80.0)
        self.declare_parameter('pick_z',           -380.0)
        self.declare_parameter('drop_x',             80.0)
        self.declare_parameter('drop_y',              0.0)
        self.declare_parameter('drop_z',           -380.0)
        self.declare_parameter('traj_v_max_mm_s',    80.0)
        self.declare_parameter('traj_a_max_mm_s2',  200.0)
        self.declare_parameter('traj_dt_s',           0.05)
        self.declare_parameter('pneumatic_can_id',        4)
        self.declare_parameter('speed_mode',            'medium')
        self.declare_parameter('gripper_length_mm',     120.0)
        self.declare_parameter('approach_z_offset_mm', 80.0)
        self.declare_parameter('random_pick_points',    [0.0, 50.0, -350.0])
        self.declare_parameter('random_place_points',   [0.0, -80.0, -300.0])

        # ── resolve speed preset (overrides traj_v/a params when set) ────
        mode = self.get_parameter('speed_mode').value
        if mode in SPEED_PRESETS:
            v, a, mv, ma, vd = SPEED_PRESETS[mode]
            self.set_parameters([
                rclpy.parameter.Parameter('traj_v_max_mm_s',  rclpy.parameter.Parameter.Type.DOUBLE, v),
                rclpy.parameter.Parameter('traj_a_max_mm_s2', rclpy.parameter.Parameter.Type.DOUBLE, a),
            ])
            self.get_logger().info(
                f'Speed mode: {mode!r}  '
                f'v={v} mm/s  a={a} mm/s²  '
                f'motor vel={mv} rad/s  acc={ma} rad/s²  '
                f'verify_delay={vd} s'
            )
        else:
            self.get_logger().warn(
                f'Unknown speed_mode {mode!r} — valid: {list(SPEED_PRESETS)}. '
                f'Using traj_v_max_mm_s / traj_a_max_mm_s2 params as-is.'
            )
            v  = self.get_parameter('traj_v_max_mm_s').value
            a  = self.get_parameter('traj_a_max_mm_s2').value
            mv, ma, vd = 1.0, 2.0, 0.30

        # ── motor controller ──────────────────────────────────────────────
        self._ctrl = DeltaMotorController(can_port='can0', vel_max=mv, acc_set=ma, verify_delay=vd)
        self._traj_v_max = v
        self._traj_a_max = a
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

        # ── subscriptions ────────────────────────────────────────────────
        # manual target from UI (mm, static object)
        self._sub = self.create_subscription(
            PointStamped, '/delta/blind_target', self._target_cb, 10)
        # camera target (meters, converted internally; prediction applied)
        self._sub_cam = self.create_subscription(
            PointStamped, '/delta/target_xyz', self._camera_target_cb, 10)
        # conveyor velocity from camera (mm/s, point.y = vy)
        self._sub_vel = self.create_subscription(
            PointStamped, '/delta/object_velocity_mm_s', self._velocity_cb, 10)
        # multi-target array from camera (meters, robot base frame)
        self._sub_all = self.create_subscription(
            PoseArray, '/delta/all_targets', self._all_targets_cb, 10)

        # ── service: stop random cycling ─────────────────────────────────
        self._srv_random = self.create_service(
            Empty, '/delta/trigger_random', self._trigger_random_cb)
        self._srv_stop = self.create_service(
            Empty, '/delta/stop_random', self._stop_random_cb)

        # ── random point lists (set via UI / set_parameters) ──────────────
        self._pick_points  = []   # list of (x, y, z)
        self._place_points = []   # list of (x, y, z)

        # ── conveyor velocity cache ───────────────────────────────────────
        self._vy = 0.0   # mm/s along Y axis, updated by camera

        # ── multi-target snapshot (latest PoseArray from camera, in mm) ──
        self._latest_targets: list = []
        self._targets_lock = threading.Lock()

        # ── FSM state ─────────────────────────────────────────────────────
        self._busy = False
        self._lock = threading.Lock()
        self._random_stop = threading.Event()

        # ── parameter callback — re-apply motor speed on speed_mode change ─
        self.add_on_set_parameters_callback(self._on_params_changed)

        self._publish_state('IDLE')
        self.get_logger().info('BlindPickAndPlace ready — waiting for trigger.')

    # ── parameter callback ────────────────────────────────────────────────────

    def _on_params_changed(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'speed_mode' and p.value in SPEED_PRESETS:
                v, a, mv, ma, vd = SPEED_PRESETS[p.value]
                self._traj_v_max = v
                self._traj_a_max = a
                self._ctrl.VEL_MAX = mv
                self._ctrl.ACC_SET = ma
                self._ctrl.VERIFY_DELAY = vd
                if config.ENABLE_MOTORS:
                    self._ctrl.setup_pp_mode()
                self.get_logger().info(
                    f'Speed mode → {p.value!r}  '
                    f'v={v} mm/s  a={a} mm/s²  motor vel={mv} rad/s  acc={ma} rad/s²'
                )
            elif p.name == 'random_pick_points':
                flat = list(p.value)
                if len(flat) % 3 == 0 and len(flat) >= 3:
                    self._pick_points = [
                        (flat[i], flat[i+1], flat[i+2]) for i in range(0, len(flat), 3)
                    ]
                    self.get_logger().info(f'Pick points updated: {self._pick_points}')
            elif p.name == 'random_place_points':
                flat = list(p.value)
                if len(flat) % 3 == 0 and len(flat) >= 3:
                    self._place_points = [
                        (flat[i], flat[i+1], flat[i+2]) for i in range(0, len(flat), 3)
                    ]
                    self.get_logger().info(f'Place points updated: {self._place_points}')
        return SetParametersResult(successful=True)

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
        """Manual UI target (mm, static object — no prediction)."""
        pos = (msg.point.x, msg.point.y, msg.point.z)
        if not check_workspace(*pos):
            self.get_logger().warn(
                f'Blind target ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) mm '
                f'is outside workspace — ignored')
            return
        self._maybe_start(pos)

    def _velocity_cb(self, msg: PointStamped) -> None:
        """Cache the latest conveyor velocity from the camera (mm/s, Y axis)."""
        self._vy = msg.point.y

    def _camera_target_cb(self, msg: PointStamped) -> None:
        """Camera detection target: meters → mm, predict Y forward by travel time."""
        x_mm = msg.point.x * 1000.0
        y_mm = msg.point.y * 1000.0
        z_mm = msg.point.z * 1000.0

        if not check_workspace(x_mm, y_mm, z_mm):
            return

        # predict where the object will be when the gripper arrives
        gl   = self.get_parameter('gripper_length_mm').value
        zoff = self.get_parameter('approach_z_offset_mm').value
        home = (HOME_X, HOME_Y, HOME_Z)
        eff_z = z_mm + gl

        v_max = self._traj_v_max
        a_max = self._traj_a_max
        above = (x_mm, y_mm, min(eff_z + zoff, config.Z_MAX))

        # first-pass travel time estimate: home → above_pick → pick
        t_est = (self._travel_time(home, above, v_max, a_max) +
                 self._travel_time(above, (x_mm, y_mm, eff_z), v_max, a_max))

        # one refinement: project Y and recompute
        y_pred   = y_mm + self._vy * t_est
        above2   = (x_mm, y_pred, max(eff_z + zoff, config.Z_MAX))
        t_est    = (self._travel_time(home, above2, v_max, a_max) +
                    self._travel_time(above2, (x_mm, y_pred, eff_z), v_max, a_max))
        y_pred   = y_mm + self._vy * t_est

        predicted = (x_mm, y_pred, z_mm)
        if not check_workspace(*predicted):
            self.get_logger().warn(
                f'Predicted pick Y={y_pred:.1f} mm leaves workspace — skipped '
                f'(vy={self._vy:.0f} mm/s  t={t_est:.3f} s)')
            return

        self.get_logger().info(
            f'Camera pick: detected Y={y_mm:.1f}  predicted Y={y_pred:.1f}'
            f'  vy={self._vy:.0f} mm/s  t={t_est:.3f} s')
        self._maybe_start(predicted)

    @staticmethod
    def _travel_time(frm, to, v_max: float, a_max: float) -> float:
        """Trapezoidal profile travel time between two 3-D points."""
        d = math.sqrt(sum((b - a) ** 2 for a, b in zip(frm, to)))
        if d < 1e-6:
            return 0.0
        d_acc = v_max ** 2 / a_max          # distance covered during accel + decel
        if d >= d_acc:
            return d / v_max + v_max / a_max
        return 2.0 * math.sqrt(d / a_max)   # short move: no cruise phase

    def _trigger_random_cb(self, _req, resp):
        if not self._pick_points or not self._place_points:
            self.get_logger().warn(
                'Random trigger ignored — no pick/place points set. '
                'Use the UI to define points first.')
            return resp
        self._random_stop.clear()
        threading.Thread(target=self._run_random_cycles, daemon=True).start()
        return resp

    def _stop_random_cb(self, _req, resp):
        self._random_stop.set()
        self.get_logger().info('Random cycling stop requested.')
        return resp

    # ── multi-target callbacks ─────────────────────────────────────────────────

    def _all_targets_cb(self, msg: PoseArray) -> None:
        """Cache all camera-confirmed targets (meters → mm) and dispatch if idle."""
        targets = [
            {'x_mm': p.position.x * 1000.0,
             'y_mm': p.position.y * 1000.0,
             'z_mm': p.position.z * 1000.0}
            for p in msg.poses
        ]
        with self._targets_lock:
            self._latest_targets = targets
        self._dispatch_next()

    def _predict_pick(self, x_mm: float, y_mm: float, z_mm: float):
        """Apply conveyor-travel prediction; return predicted (x, y, z) mm or None."""
        gl    = self.get_parameter('gripper_length_mm').value
        zoff  = self.get_parameter('approach_z_offset_mm').value
        v_max = self._traj_v_max
        a_max = self._traj_a_max
        home  = (HOME_X, HOME_Y, HOME_Z)
        eff_z = z_mm + gl
        approach_z = min(eff_z + zoff, config.Z_MAX)

        above  = (x_mm, y_mm, approach_z)
        t_est  = (self._travel_time(home, above, v_max, a_max) +
                  self._travel_time(above, (x_mm, y_mm, eff_z), v_max, a_max))
        y_pred = y_mm + self._vy * t_est

        above2 = (x_mm, y_pred, approach_z)
        t_est  = (self._travel_time(home, above2, v_max, a_max) +
                  self._travel_time(above2, (x_mm, y_pred, eff_z), v_max, a_max))
        y_pred = y_mm + self._vy * t_est

        predicted = (x_mm, y_pred, z_mm)
        if not check_workspace(*predicted):
            return None
        return predicted

    def _dispatch_next(self) -> None:
        """When idle, pick the highest-priority target from the latest camera snapshot.

        Priority = "first to exit workspace":
          - Conveyor moves in -Y → object with lowest predicted Y exits first.
          - Static/slow belt → closest to HOME goes first (minimises travel time).
        """
        with self._targets_lock:
            targets = list(self._latest_targets)

        if not targets:
            return

        candidates = []
        for t in targets:
            predicted = self._predict_pick(t['x_mm'], t['y_mm'], t['z_mm'])
            if predicted is None:
                continue
            vy = self._vy
            if abs(vy) > 1.0:
                # "first to exit" — most extreme Y in direction of belt travel wins
                priority = predicted[1] * (1.0 if vy >= 0.0 else -1.0)
            else:
                # static: closest to home
                priority = -math.sqrt(sum((a - b) ** 2 for a, b in
                                         zip(predicted, (HOME_X, HOME_Y, HOME_Z))))
            candidates.append((priority, predicted))

        if not candidates:
            return

        candidates.sort(key=lambda c: c[0], reverse=True)
        _, pick_xyz = candidates[0]
        self.get_logger().info(
            f'[DISPATCH]  {len(candidates)} valid target(s) — '
            f'picking ({pick_xyz[0]:.1f}, {pick_xyz[1]:.1f}, {pick_xyz[2]:.1f}) mm'
        )
        self._maybe_start(pick_xyz)

    def _run_random_cycles(self):
        log = self.get_logger()
        log.info(
            f'Random cycling started — '
            f'{len(self._pick_points)} pick pts, '
            f'{len(self._place_points)} place pts')
        while not self._random_stop.is_set():
            pick  = random.choice(self._pick_points)
            place = random.choice(self._place_points)
            log.info(f'Random: pick={pick}  place={place}')
            self._run_one(pick, place)
            if self._random_stop.is_set():
                break
        log.info('Random cycling stopped.')

    # ── sequence management ───────────────────────────────────────────────────

    def _maybe_start(self, pick_xyz) -> None:
        with self._lock:
            if self._busy:
                self.get_logger().warn(
                    'Already executing a sequence — new trigger dropped')
                return
            self._busy = True
        p = self.get_parameter
        place = (p('drop_x').value, p('drop_y').value, p('drop_z').value)
        threading.Thread(
            target=self._run_one, args=(pick_xyz, place), daemon=True
        ).start()

    def _run_one(self, pick_xyz, place_xyz) -> None:
        """Single pick-and-place cycle.

        Transit strategy: always move through the centre column (HOME_X, HOME_Y)
        at a transit Z that is deep enough for the full workspace to be open (≥200 mm
        radius in every direction).  This avoids the non-convex ceiling gaps that
        make diagonal paths unreachable when pick/place are far from centre.

        Path:
          HOME
            ↓ (HOME_X, HOME_Y, transit_z)    — descend at centre
            → (pick_x,  pick_y,  transit_z)  — lateral traverse at safe depth
            ↓ pick_xyz                        — final descent (if deeper than transit)
          GRASP
            ↑ (pick_x,  pick_y,  transit_z)  — rise back to transit Z
            → (HOME_X,  HOME_Y,  transit_z)  — return to centre column
            → (place_x, place_y, transit_z)  — traverse to place
            ↓ place_xyz                       — final descent
          DROP
            ↑ (place_x, place_y, transit_z)  — rise
            → (HOME_X,  HOME_Y,  transit_z)  — centre column
            ↑ HOME                            — ascend to home
        """
        log   = self.get_logger()
        v_max = self._traj_v_max
        a_max = self._traj_a_max
        dt    = self.get_parameter('traj_dt_s').value
        home  = (HOME_X, HOME_Y, HOME_Z)

        # At Z ≤ -400 the workspace radius is ≥ 200 mm in every direction,
        # so any lateral move at transit_z is safe.
        SAFE_TRANSIT_Z = -400.0
        transit_z = min(pick_xyz[2], place_xyz[2], SAFE_TRANSIT_Z)

        home_t  = (HOME_X,      HOME_Y,      transit_z)
        pick_t  = (pick_xyz[0], pick_xyz[1], transit_z)
        place_t = (place_xyz[0], place_xyz[1], transit_z)

        with self._lock:
            self._busy = True

        def move(label, frm, to):
            wps = linear_waypoints(frm, to, v_max=v_max, a_max=a_max, dt=dt)
            if not self._ctrl.execute_trajectory(wps, dt=dt):
                raise RuntimeError(f'{label} trajectory did not reach target')

        try:
            self._publish_state('MOVING_TO_PICK')
            log.info(
                f'[MOVING_TO_PICK]  {home} → {home_t} → {pick_t} → {pick_xyz}')
            move('home→home_t',   home,   home_t)
            move('home_t→pick_t', home_t, pick_t)
            move('pick_t→pick',   pick_t, pick_xyz)

            self._publish_state('GRASPING')
            log.info('[GRASPING]  closing gripper')
            if config.ENABLE_MOTORS:
                self._gripper.close(wait=True)

            self._publish_state('MOVING_TO_PLACE')
            log.info(
                f'[MOVING_TO_PLACE]  {pick_xyz} → {pick_t} → {home_t} → {place_t} → {place_xyz}')
            move('pick→pick_t',    pick_xyz, pick_t)
            move('pick_t→home_t',  pick_t,   home_t)
            move('home_t→place_t', home_t,   place_t)
            move('place_t→place',  place_t,  place_xyz)

            self._publish_state('DROPPING')
            log.info('[DROPPING]  opening gripper')
            if config.ENABLE_MOTORS:
                self._gripper.open(wait=True)

        except Exception as exc:
            log.error(f'Sequence aborted: {type(exc).__name__}: {exc}')
            if config.ENABLE_MOTORS:
                self._gripper.open(wait=False)

        finally:
            self._publish_state('RESETTING')
            try:
                ok, cx, cy, cz = self._ctrl.get_current_xyz()
                current = (cx, cy, cz) if ok else home_t
                cur_t   = (current[0], current[1], transit_z)
                log.info(f'[RESETTING]  returning home via transit_z={transit_z}')
                move('reset→cur_t', current, cur_t)
                move('cur_t→home_t', cur_t,   home_t)
                move('home_t→home',  home_t,  home)
            except Exception as exc:
                log.error(f'RESETTING failed: {exc}')

            with self._lock:
                self._busy = False
            self._publish_state('IDLE')
            log.info('Cycle complete — IDLE')
            self._dispatch_next()

    # ── helpers ───────────────────────────────────────────────────────────────

    def _publish_state(self, state: str) -> None:
        m = String()
        m.data = state
        self._state_pub.publish(m)

    def destroy_node(self) -> None:
        if config.ENABLE_MOTORS:
            try:
                self._gripper.open(wait=False)
                self._ctrl.move_xyz(HOME_X, HOME_Y, HOME_Z, raw=True)
                time.sleep(0.5)
            except Exception:
                pass
            self._gripper.disconnect()
            self._ctrl.shutdown()
        super().destroy_node()


# ═════════════════════════════════════════════════════════════════════════════
def main(args=None):
    import signal
    rclpy.init(args=args)
    node = BlindPickAndPlace()

    def handle_sigint(sig, frame):
        node.get_logger().info("Ctrl+C received — shutting down cleanly")
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_sigint)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
