#!/usr/bin/env python3
"""
matlab_bridge_node.py — Delta robot MATLAB IK bridge with built-in FSM.

Receives a confirmed target XYZ from the camera pipeline, packages it into
a DeltaTarget custom message, and sends it to MATLAB for IK solving.
MATLAB publishes the joint angles back as DeltaJointAngles.  The node
validates and executes the move, then returns home.

━━━━ ROS2 ↔ MATLAB topic map ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Camera → Bridge  (subscribe):
      /delta/target_xyz            geometry_msgs/PointStamped
          x/y/z in mm, robot base frame, EE-tip Z

  Bridge → MATLAB  (publish):
      /delta/matlab/target_xyz     custom_messages/DeltaTarget
          x_mm, y_mm, z_mm, confidence, track_id, detection_mode

  MATLAB → Bridge  (subscribe):
      /delta/matlab/joint_thetas   custom_messages/DeltaJointAngles
          theta1_deg, theta2_deg, theta3_deg, ik_valid

  Bridge → All     (publish):
      /delta/matlab/bridge_state   std_msgs/String   — current FSM state
      /delta/matlab/fk_result      geometry_msgs/PointStamped — FK after move

━━━━ MATLAB code (Robotics System Toolbox ≥ R2022b) ━━━━━━━━━━━━━━━━━━━
  node = ros2node("/matlab_ik");
  sub  = ros2subscriber(node, "/delta/matlab/target_xyz",
                        "custom_messages/DeltaTarget");
  pub  = ros2publisher(node, "/delta/matlab/joint_thetas",
                       "custom_messages/DeltaJointAngles");
  while true
      tgt = receive(sub, 10);              % 10 s timeout
      x = tgt.x_mm;  y = tgt.y_mm;  z = tgt.z_mm;
      [t1, t2, t3, valid] = my_delta_ik(x, y, z);
      reply = ros2message("custom_messages/DeltaJointAngles");
      reply.theta1_deg = t1;
      reply.theta2_deg = t2;
      reply.theta3_deg = t3;
      reply.ik_valid   = valid;
      send(pub, reply);
  end

━━━━ State machine ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IDLE → WAITING_MATLAB → APPROACHING → MOVING → RESETTING → IDLE
                        ↘ ERROR  ──────────────────────────────────↗
"""

import collections
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from custom_messages.msg import DeltaTarget, DeltaJointAngles
from delta_common import config
from delta_common.fk_ik import delta_calcForward, e, f, re, rf
from delta_motor_controller.motor_controller import DeltaMotorController

MATLAB_TIMEOUT_S     = 3.0    # seconds to wait for MATLAB joint-angle reply
APPROACH_Z_OFFSET_MM = 40.0   # mm above EE-tip pick Z to hover before descending
APPROACH_SETTLE_SEC  = 0.05   # settle time at approach height before pick
Z_DROP_EXTRA_MM      = -15.0  # extra descent commanded to MATLAB, pick and place (deeper = more negative)

# ── Terrain-robust Z estimation ───────────────────────────────────────────────
# Vegetation causes IR depth holes directly over plants.  We sample an annular
# ring around the centroid (skipping the plant centre) to hit soil/root level,
# then bias toward the deeper (farther-from-camera) returns which represent the
# ground surface rather than raised leaf/stem tips.
Z_ANNULUS_INNER_R_PX  = 8     # exclude this radius around centroid (plant top / hole)
Z_ANNULUS_OUTER_R_PX  = 40    # outer radius of soil-sample ring (px)
Z_SOIL_PERCENTILE     = 50.0  # median of surface returns (robust to grass tips AND background)
Z_BG_REJECT_MM        = 50.0  # reject ring returns > this depth below the ring minimum
Z_MIN_VALID_PX        = 5     # need at least this many valid depth pixels
# Temporal: keep a per-target rolling Z buffer to suppress frame-to-frame noise.
# Buffer resets when the target XY jumps >50 mm (different plant).
Z_HISTORY_MAXLEN      = 6
Z_HISTORY_RESET_MM    = 50.0  # XY distance that triggers a buffer reset (mm)
# Safety: maximum depth MATLAB's IK result may command beyond the detected target Z.
# If the FK of MATLAB's thetas puts the EE-tip more than this below the detected
# surface, the move is rejected before any motors move.
# Includes headroom for the intentional Z_DROP_EXTRA_MM offset above, plus the
# original 20mm anomaly margin for MATLAB's own correction.
MATLAB_Z_GUARD_MM     = 20.0 + abs(Z_DROP_EXTRA_MM)
# Hard absolute floor for EE-tip Z (mm, robot base frame, negative = below base).
# Crash confirmed at EE-tip < -640 mm.  Set 5 mm above that as the kill limit.
# Checked on BOTH the incoming detected target AND the MATLAB-commanded position.
EETIP_Z_FLOOR_MM      = -638.0


# ── State machine ──────────────────────────────────────────────────────────────

class MatlabBridgeFSM:
    """
    Pure FSM — no ROS dependency; driven by the ROS node via public methods.

    States
    ------
    IDLE            : waiting for a target
    WAITING_MATLAB  : DeltaTarget published; waiting for DeltaJointAngles back
    APPROACHING     : moving to hover height above pick Z using local IK (bg thread)
    MOVING          : executing the MATLAB theta solution on the motors (bg thread)
    RESETTING       : returning to home position after a move (bg thread)
    ERROR           : IK invalid or MATLAB timeout; recovering to IDLE (bg thread)
    """

    def __init__(self, controller: DeltaMotorController, publish_state_fn, publish_fk_fn,
                 publish_motor_thetas_fn, logger):
        self._ctrl            = controller
        self._pub_state       = publish_state_fn         # fn(state: str)
        self._pub_fk          = publish_fk_fn             # fn(x, y, z)
        self._pub_motor_thetas = publish_motor_thetas_fn   # fn(fb_deg, ok)
        self._log             = logger

        self._state  = "IDLE"
        self._busy   = False
        self._lock   = threading.Lock()

        self._target_xyz    = None   # (x, y, z) mm
        self._matlab_thetas = None   # (t1, t2, t3) deg
        self._wait_start    = 0.0

    # ── public API ─────────────────────────────────────────────────────────────

    @property
    def state(self) -> str:
        return self._state

    def on_target(self, x: float, y: float, z: float) -> bool:
        """Accept a new target.  Returns False if the robot is busy."""
        with self._lock:
            if self._busy:
                return False
            self._target_xyz    = (x, y, z)
            self._matlab_thetas = None
            self._busy          = True
            self._wait_start    = time.time()
            self._set_state("WAITING_MATLAB")
            return True

    def update_target(self, x: float, y: float, z: float) -> bool:
        """Refresh the pending target with a fresher detection while still
        WAITING_MATLAB or APPROACHING, so Phase 2 can commit to the most
        recent reading instead of the one captured at cycle start.
        Returns True if the target was refreshed."""
        with self._lock:
            if self._state in ("WAITING_MATLAB", "APPROACHING"):
                self._target_xyz = (x, y, z)
                return True
            return False

    def on_matlab_reply(self, t1: float, t2: float, t3: float, ik_valid: bool) -> None:
        """Handle DeltaJointAngles from MATLAB."""
        with self._lock:
            if self._state != "WAITING_MATLAB":
                return
            if not ik_valid:
                self._log.error(
                    f"MATLAB reported ik_valid=False for target "
                    f"({self._target_xyz[0]:.1f},{self._target_xyz[1]:.1f},"
                    f"{self._target_xyz[2]:.1f}) — aborting"
                )
                self._set_state("ERROR")
                threading.Thread(target=self._recover, daemon=True).start()
                return
            self._matlab_thetas = (t1, t2, t3)
            self._set_state("APPROACHING")
            threading.Thread(target=self._run_move, daemon=True).start()

    def tick(self) -> None:
        """Call at ~10 Hz to enforce the MATLAB response timeout."""
        with self._lock:
            if self._state != "WAITING_MATLAB":
                return
            elapsed = time.time() - self._wait_start
            if elapsed > MATLAB_TIMEOUT_S:
                self._log.error(
                    f"MATLAB timeout ({elapsed:.1f} s > {MATLAB_TIMEOUT_S} s) — "
                    f"no reply for target "
                    f"({self._target_xyz[0]:.1f},{self._target_xyz[1]:.1f},"
                    f"{self._target_xyz[2]:.1f})"
                )
                self._set_state("ERROR")
                threading.Thread(target=self._recover, daemon=True).start()

    # ── background threads ─────────────────────────────────────────────────────

    def _run_move(self) -> None:
        t1, t2, t3 = self._matlab_thetas

        # Phase 1: hover above pick position using local IK so the robot
        # never dives straight to pick depth without a safe intermediate point.
        x_m, y_m, z_m = self._target_xyz          # meters, EE-tip frame
        x_mm  = x_m  * 1000.0
        y_mm  = y_m  * 1000.0
        z_approach_mm = z_m * 1000.0 + APPROACH_Z_OFFSET_MM   # EE-tip Z

        # Hard clamp: approach must never breach the absolute floor.
        if z_approach_mm < EETIP_Z_FLOOR_MM:
            self._log.error(
                f"Approach EE-tip={z_approach_mm:.1f}mm < floor={EETIP_Z_FLOOR_MM}mm "
                "— aborting before approach"
            )
            self._go_error()
            return

        self._log.info(
            f"Approach: XY=({x_mm:.1f},{y_mm:.1f}) "
            f"z_eetip={z_approach_mm:.1f}mm  floor={EETIP_Z_FLOOR_MM}mm"
        )
        # raw=False → move_xyz adds EE_OFFSET_Z_MM internally (EE-tip → platform)
        ok_app, *_ = self._ctrl.move_xyz(x_mm, y_mm, z_approach_mm)
        if not ok_app:
            self._log.warn(
                f"Approach move failed at z_eetip={z_approach_mm:.1f}mm"
            )
        time.sleep(APPROACH_SETTLE_SEC)

        # Phase 2: descend to MATLAB-computed pick position.
        # Re-sample the target right before committing — update_target() has
        # been refreshing self._target_xyz with live detections throughout
        # WAITING_MATLAB/APPROACHING, so this picks up the freshest reading
        # instead of the one frozen at cycle start.
        with self._lock:
            x_resampled, y_resampled, z_resampled = self._target_xyz
            self._set_state("MOVING")
        if abs(z_resampled - z_m) * 1000.0 > 1.0:
            self._log.info(
                f"Target re-sampled before descend: "
                f"z_eetip {z_m * 1000.0:.1f}mm → {z_resampled * 1000.0:.1f}mm "
                f"(Δ={(z_resampled - z_m) * 1000.0:+.1f}mm)"
            )
        x_m, y_m, z_m = x_resampled, y_resampled, z_resampled

        self._log.info(
            f"Executing MATLAB thetas: θ1={t1:.2f}° θ2={t2:.2f}° θ3={t3:.2f}°"
        )

        if not self._ctrl.within_joint_limits(t1, t2, t3):
            self._log.error(
                f"MATLAB thetas outside joint limits "
                f"(limits θ_max={config.THETA1_MAX}°): "
                f"({t1:.1f},{t2:.1f},{t3:.1f}) — aborting"
            )
            self._go_error()
            return

        # ── Z guard ───────────────────────────────────────────────────────────
        # Compute FK of MATLAB's thetas BEFORE sending to motors.
        # If the resulting EE-tip Z is more than MATLAB_Z_GUARD_MM below the
        # detected target surface, the move is rejected — MATLAB's correction
        # must not drive the gripper through the belt/terrain.
        st_pre, _, _, z_pre = delta_calcForward(t1, t2, t3, e, f, re, rf)
        if st_pre == 0:
            z_eetip_commanded = z_pre - config.EE_OFFSET_Z_MM
            z_eetip_target    = self._target_xyz[2] * 1000.0   # detected, mm
            overrun = z_eetip_target - z_eetip_commanded        # positive = commanded deeper
            self._log.info(
                f"Z guard: EE-tip commanded={z_eetip_commanded:.1f}mm  "
                f"target={z_eetip_target:.1f}mm  overrun={overrun:+.1f}mm  "
                f"floor={EETIP_Z_FLOOR_MM}mm"
            )
            if z_eetip_commanded < EETIP_Z_FLOOR_MM:
                self._log.error(
                    f"Z floor TRIP: MATLAB commands EE-tip={z_eetip_commanded:.1f}mm "
                    f"< floor={EETIP_Z_FLOOR_MM}mm — aborting"
                )
                self._go_error()
                return
            if overrun > MATLAB_Z_GUARD_MM:
                self._log.error(
                    f"Z guard TRIP: MATLAB commands EE-tip {overrun:.1f}mm below "
                    f"detected surface (guard={MATLAB_Z_GUARD_MM}mm) — aborting"
                )
                self._go_error()
                return
        else:
            self._log.error(
                f"Z guard: FK of MATLAB thetas ({t1:.1f},{t2:.1f},{t3:.1f})° "
                "returned no solution — aborting"
            )
            self._go_error()
            return

        ok, fk_xyz, err, fb_deg = self._ctrl.move_thetas(t1, t2, t3)
        if fk_xyz is not None:
            self._pub_fk(fk_xyz[0], fk_xyz[1], fk_xyz[2])
        self._pub_motor_thetas(fb_deg, ok)

        if ok:
            self._log.info(
                f"Move OK  FK=({fk_xyz[0]:.1f},{fk_xyz[1]:.1f},{fk_xyz[2]:.1f})"
                f"  err={err:.2f} mm"
            )
        else:
            self._log.warn(f"Move settled with err={err:.2f} mm (above tolerance)")

        with self._lock:
            self._set_state("RESETTING")
        self._do_reset()

    def _do_reset(self) -> None:
        ok, fk_xyz, _, fb_deg = self._ctrl.move_thetas(0.0, 0.0, 0.0)   # exact θ=0,0,0 home
        if ok and fk_xyz is not None:
            self._pub_fk(fk_xyz[0], fk_xyz[1], fk_xyz[2])
        self._pub_motor_thetas(fb_deg, ok)
        with self._lock:
            self._state         = "IDLE"
            self._busy          = False
            self._target_xyz    = None
            self._matlab_thetas = None
        self._pub_state("IDLE")
        self._log.info("[FSM] → IDLE")

    def _go_error(self) -> None:
        with self._lock:
            self._set_state("ERROR")
        self._recover()

    def _recover(self) -> None:
        time.sleep(1.0)
        try:
            self._ctrl.move_thetas(0.0, 0.0, 0.0)   # exact θ=0,0,0 home
        except Exception as exc:
            self._log.error(f"Home move during recovery failed: {exc}")
        with self._lock:
            self._state         = "IDLE"
            self._busy          = False
            self._target_xyz    = None
            self._matlab_thetas = None
        self._pub_state("IDLE")
        self._log.info("[FSM] → IDLE (recovered from ERROR)")

    def _set_state(self, state: str) -> None:
        self._state = state
        self._pub_state(state)
        self._log.info(f"[FSM] → {state}")


# ── ROS2 node ──────────────────────────────────────────────────────────────────

class MatlabBridgeNode(Node):
    """
    Bridges the delta robot camera pipeline with a MATLAB IK solver.

    Run alongside the camera node (NOT alongside delta_main_app — they both
    consume /delta/target_xyz and command the same motors).
    """

    def __init__(self):
        super().__init__("matlab_bridge_node")

        # ── motor controller ──────────────────────────────────────────────────
        self._ctrl = DeltaMotorController(
            can_port="can0",
            vel_max=config.MOTOR_VEL_MAX,
            acc_set=config.MOTOR_ACC_SET,
        )
        self._explicit_shutdown = False   # True only on intentional Ctrl+C
        if config.ENABLE_MOTORS:
            self._ctrl.connect()
            self.get_logger().info("Motors connected.")
        else:
            self.get_logger().warn("ENABLE_MOTORS=False — dry-run mode")

        # ── publishers ────────────────────────────────────────────────────────
        self._pub_to_matlab = self.create_publisher(
            DeltaTarget, "/delta/matlab/target_xyz", 10
        )
        self._pub_state = self.create_publisher(
            String, "/delta/matlab/bridge_state", 10
        )
        self._pub_fk = self.create_publisher(
            PointStamped, "/delta/matlab/fk_result", 10
        )
        # Actual motor-encoder thetas (CAN feedback), read back after every
        # move and republished to MATLAB — separate from the commanded
        # /delta/matlab/joint_thetas MATLAB sends us.
        self._pub_motor_thetas = self.create_publisher(
            DeltaJointAngles, "/delta/matlab/motor_thetas", 10
        )
        # Shared EE-tip FK — read by camera_system
        self._pub_ee_fk = self.create_publisher(
            PointStamped, "/delta/ee_fk_xyz", 10
        )
        # EE position for MATLAB's predict_target block — must be published
        # so /delta/ee_position_mm has a real source when running this bridge
        # (main_app publishes the same data via /delta/matlab/ee_position_mm,
        # but MATLAB's model subscribes to /delta/ee_position_mm).
        self._pub_ee_pos_mm = self.create_publisher(
            PointStamped, "/delta/ee_position_mm", 10
        )

        # ── FSM ───────────────────────────────────────────────────────────────
        self._fsm = MatlabBridgeFSM(
            controller=self._ctrl,
            publish_state_fn=self._send_state,
            publish_fk_fn=self._send_fk,
            publish_motor_thetas_fn=self._send_motor_thetas,
            logger=self.get_logger(),
        )

        # ── RealSense depth ───────────────────────────────────────────────────
        # Camera intrinsics (filled on first CameraInfo message)
        self._fx = self._fy = self._cx = self._cy = None

        # Pre-build camera transforms from config so we can reverse-project
        # base-frame coords back to pixel and then re-deproject with real depth.
        R = np.array(config.CAMERA_DIRECT_MATRIX, dtype=np.float64)
        t = np.array([config.CAM_TX_MM, config.CAM_TY_MM, config.CAM_TZ_MM],
                     dtype=np.float64)
        self._T_cam_to_base = np.eye(4, dtype=np.float64)
        self._T_cam_to_base[:3, :3] = R
        self._T_cam_to_base[:3, 3] = t
        self._T_base_to_cam = np.eye(4, dtype=np.float64)
        self._T_base_to_cam[:3, :3] = R.T
        self._T_base_to_cam[:3, 3] = -R.T @ t

        self._depth_lock = threading.Lock()
        self._depth_image = None   # H×W uint16, each count = 1 mm

        # ── Terrain Z temporal smoother ───────────────────────────────────────
        # Accumulates per-target Z readings and outputs a rolling median to
        # suppress frame-to-frame depth noise over vegetation.  Resets when the
        # detected XY jumps > Z_HISTORY_RESET_MM (different plant).
        self._z_history: collections.deque = collections.deque(maxlen=Z_HISTORY_MAXLEN)
        self._z_history_xy: tuple | None = None   # (x_m, y_m) of buffered readings

        # ── subscribers ───────────────────────────────────────────────────────
        self._sub_target = self.create_subscription(
            PointStamped,
            "/delta/target_xyz",
            self._on_target_xyz,
            10,
        )
        self._sub_matlab = self.create_subscription(
            DeltaJointAngles,
            "/delta/matlab/joint_thetas",
            self._on_joint_angles,
            10,
        )
        self._sub_depth = self.create_subscription(
            Image,
            config.DEPTH_TOPIC,
            self._on_depth_image,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._sub_cam_info = self.create_subscription(
            CameraInfo,
            config.CAMERA_INFO_TOPIC,
            self._on_camera_info,
            1,
        )

        # ── 10 Hz tick for MATLAB timeout ─────────────────────────────────────
        self.create_timer(0.1, self._fsm.tick)
        # ── 5 Hz FK heartbeat — keeps camera_system's ee_fk_xyz fresh ─────────
        self.create_timer(0.2, self._publish_fk_heartbeat)

        self.get_logger().info(
            "MatlabBridgeNode ready\n"
            "  Listening : /delta/target_xyz\n"
            "  → MATLAB  : /delta/matlab/target_xyz   (DeltaTarget)\n"
            "  ← MATLAB  : /delta/matlab/joint_thetas (DeltaJointAngles)\n"
            "  State     : /delta/matlab/bridge_state\n"
            f"  MATLAB timeout: {MATLAB_TIMEOUT_S} s"
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_target_xyz(self, msg: PointStamped) -> None:
        x = msg.point.x   # metres, robot base frame
        y = msg.point.y
        z = msg.point.z

        # Replace z with real RealSense depth when available
        real = self._reproject_with_real_depth(x, y, z)
        if real is not None:
            x_real, y_real, z_real = real
            self.get_logger().info(
                f"RealSense depth: z_fake={z * 1000.0:.1f} mm → "
                f"z_real={z_real * 1000.0:.1f} mm  "
                f"(Δ={abs(z_real - z) * 1000.0:.1f} mm)"
            )
            x, y, z = x_real, y_real, z_real
        else:
            self.get_logger().warn(
                "RealSense depth unavailable — using z from /delta/target_xyz "
                f"(z={z * 1000.0:.1f} mm)"
            )

        # Terrain temporal smoothing — median over recent Z readings for this
        # plant location.  Suppresses frame-to-frame depth flicker over vegetation.
        z_raw_mm = z * 1000.0
        z = self._smooth_z(x, y, z)
        z_smooth_mm = z * 1000.0
        if abs(z_smooth_mm - z_raw_mm) > 1.0:
            self.get_logger().info(
                f"Z smoothed: raw={z_raw_mm:.1f} mm → "
                f"smooth={z_smooth_mm:.1f} mm  "
                f"(buf={len(self._z_history)})"
            )

        if not self._fsm.on_target(x, y, z):
            if not self._fsm.update_target(x, y, z):
                self.get_logger().warn(
                    f"Target ({x * 1000.0:.1f},{y * 1000.0:.1f},{z * 1000.0:.1f}) mm "
                    f"dropped — robot busy (state={self._fsm.state})"
                )
            return

        # Convert EE-tip Z (from camera) to platform Z for the IK solver.
        # move_thetas positions the platform; the EE tip is EE_OFFSET_Z_MM below it.
        x_mm = float(x) * 1000.0
        y_mm = float(y) * 1000.0
        z_eetip_mm = float(z) * 1000.0
        z_platform_mm = (
            z_eetip_mm + config.EE_OFFSET_Z_MM + Z_DROP_EXTRA_MM
        )  # e.g. -670 + 150 - 15 = -535

        self.get_logger().info(
            f"z_eetip={z_eetip_mm:.1f} mm → z_platform={z_platform_mm:.1f} mm "
            f"(incl. {Z_DROP_EXTRA_MM:+.1f}mm extra drop)"
        )

        out = DeltaTarget()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = "robot_base"
        out.x_mm            = x_mm
        out.y_mm            = y_mm
        out.z_mm            = z_platform_mm   # platform Z — what the IK solver needs
        out.confidence      = -1.0
        out.track_id        = -1
        out.detection_mode  = config.DETECTION_MODE
        self._pub_to_matlab.publish(out)

    def _on_joint_angles(self, msg: DeltaJointAngles) -> None:
        self.get_logger().info(
            f"MATLAB reply: θ=({msg.theta1_deg:.2f},{msg.theta2_deg:.2f},"
            f"{msg.theta3_deg:.2f})° ik_valid={msg.ik_valid}"
        )
        self._fsm.on_matlab_reply(
            msg.theta1_deg, msg.theta2_deg, msg.theta3_deg, msg.ik_valid
        )

    # ── RealSense depth callbacks ─────────────────────────────────────────────

    def _on_depth_image(self, msg: Image) -> None:
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
        with self._depth_lock:
            self._depth_image = arr

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._fx is None:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]
            self.get_logger().info(
                f"Camera intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} "
                f"cx={self._cx:.1f} cy={self._cy:.1f}"
            )

    def _smooth_z(self, x_m: float, y_m: float, z_m: float) -> float:
        """Rolling median filter on Z — resets when XY position jumps to a new plant."""
        reset = False
        if self._z_history_xy is not None:
            dx = (x_m - self._z_history_xy[0]) * 1000.0
            dy = (y_m - self._z_history_xy[1]) * 1000.0
            if math.sqrt(dx ** 2 + dy ** 2) > Z_HISTORY_RESET_MM:
                self._z_history.clear()
                reset = True
        self._z_history_xy = (x_m, y_m)
        self._z_history.append(z_m)
        if reset:
            self.get_logger().debug("Z history reset — new plant location")
        return float(np.median(self._z_history))

    def _reproject_with_real_depth(self, x_m: float, y_m: float, z_m: float):
        """Replace z using real RealSense depth.

        Projects base-frame XYZ to an image pixel, samples depth in an annular
        ring around the centroid (skipping the plant-top hole) at the soil-biased
        percentile, then re-deprojects to base frame.

        Returns (x_m, y_m, z_real_m) or None if depth is unavailable.
        """
        if self._fx is None:
            return None

        with self._depth_lock:
            depth_img = self._depth_image
        if depth_img is None:
            self.get_logger().warn("RealSense depth image not yet received")
            return None

        # metres → mm, then base frame → camera frame
        p_b = np.array([x_m * 1000.0, y_m * 1000.0, z_m * 1000.0, 1.0])
        p_c = self._T_base_to_cam @ p_b
        xc, yc, zc = p_c[:3]
        if zc <= 1.0:
            return None

        u = int(round(self._fx * xc / zc + self._cx))
        v = int(round(self._fy * yc / zc + self._cy))

        h, w = depth_img.shape
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(
                f"Projected pixel ({u},{v}) outside depth image ({w}×{h})"
            )
            return None

        # ── Terrain/vegetation depth sampling ────────────────────────────────
        # Vegetation causes IR depth holes directly over plants.  Strategy:
        #   1. Sample an annular ring (skip inner Z_ANNULUS_INNER_R_PX pixels
        #      which cover the plant top / hole).  The ring captures surrounding
        #      soil/root-level returns.
        #   2. Use Z_SOIL_PERCENTILE (>50) to bias toward deeper returns — the
        #      ground surface is always deeper than raised leaf/stem tips.
        #   3. Fall back to full-patch median only if the annular ring is empty
        #      (e.g. plant too close to image border).
        z_cam_mm = None
        inner_r = Z_ANNULUS_INNER_R_PX
        outer_r = Z_ANNULUS_OUTER_R_PX

        y0 = max(0, v - outer_r);  y1 = min(h, v + outer_r + 1)
        x0 = max(0, u - outer_r);  x1 = min(w, u + outer_r + 1)
        patch = depth_img[y0:y1, x0:x1].astype(np.float32)

        # Per-pixel distance from centroid to build the annular mask.
        rows = (np.arange(y0, y1) - v)[:, None]
        cols = (np.arange(x0, x1) - u)[None, :]
        dist = np.sqrt(rows ** 2 + cols ** 2)

        # Annular ring: between inner_r and outer_r
        ring_flat  = patch[( dist >= inner_r) & (dist <= outer_r)].flatten()
        valid_ring = ring_flat[(ring_flat > 1.0) & (ring_flat < 10_000.0)]

        if len(valid_ring) >= Z_MIN_VALID_PX:
            # Reject background: values more than Z_BG_REJECT_MM beyond the
            # shallowest ring return are likely through-surface or room clutter.
            fg_ring = valid_ring[valid_ring <= valid_ring.min() + Z_BG_REJECT_MM]
            use = fg_ring if len(fg_ring) >= Z_MIN_VALID_PX else valid_ring
            z_cam_mm = float(np.percentile(use, Z_SOIL_PERCENTILE))
            self.get_logger().debug(
                f"Terrain Z (annular ring, n={len(use)}/{len(valid_ring)}): "
                f"{z_cam_mm:.1f} mm"
            )
        else:
            # Ring is empty or too sparse — fall back to full-patch median
            # (may happen near image borders or with very dense canopy).
            full_flat  = patch.flatten()
            valid_full = full_flat[(full_flat > 1.0) & (full_flat < 10_000.0)]
            if len(valid_full) >= Z_MIN_VALID_PX:
                fg_full = valid_full[valid_full <= valid_full.min() + Z_BG_REJECT_MM]
                use = fg_full if len(fg_full) >= Z_MIN_VALID_PX else valid_full
                z_cam_mm = float(np.percentile(use, Z_SOIL_PERCENTILE))
                self.get_logger().debug(
                    f"Terrain Z (full patch fallback, n={len(use)}): "
                    f"{z_cam_mm:.1f} mm"
                )
            else:
                self.get_logger().warn(
                    f"No valid depth at pixel ({u},{v}) — "
                    f"ring={len(valid_ring)} px, full={len(valid_full)} px"
                )
                return None

        x_cam_mm = (u - self._cx) * z_cam_mm / self._fx
        y_cam_mm = (v - self._cy) * z_cam_mm / self._fy

        p_real = self._T_cam_to_base @ np.array([x_cam_mm, y_cam_mm, z_cam_mm, 1.0])
        return p_real[0] / 1000.0, p_real[1] / 1000.0, p_real[2] / 1000.0

    # ── publish helpers ───────────────────────────────────────────────────────

    def _send_state(self, state: str) -> None:
        m = String()
        m.data = state
        self._pub_state.publish(m)

    def _send_motor_thetas(self, fb_deg, ok: bool) -> None:
        """Publish actual motor-encoder thetas (CAN feedback) back to MATLAB."""
        if fb_deg is None:
            return
        m = DeltaJointAngles()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "robot_base"
        m.theta1_deg = float(fb_deg[0])
        m.theta2_deg = float(fb_deg[1])
        m.theta3_deg = float(fb_deg[2])
        m.ik_valid   = bool(ok)
        self._pub_motor_thetas.publish(m)

    def _send_fk(self, x: float, y: float, z: float) -> None:
        """x, y, z are PLATFORM coordinates from delta_calcForward."""
        pt = PointStamped()
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.header.frame_id = "robot_base"
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)
        self._pub_fk.publish(pt)
        # Camera system expects EE-TIP Z on /delta/ee_fk_xyz (tip = platform - offset)
        pt_ee = PointStamped()
        pt_ee.header.stamp    = pt.header.stamp
        pt_ee.header.frame_id = "robot_base"
        pt_ee.point.x = float(x)
        pt_ee.point.y = float(y)
        pt_ee.point.z = float(z) - config.EE_OFFSET_Z_MM
        self._pub_ee_fk.publish(pt_ee)

    def _publish_fk_heartbeat(self) -> None:
        """Refresh /delta/ee_fk_xyz and /delta/ee_position_mm from live FK."""
        if not config.ENABLE_MOTORS or not self._ctrl.connected:
            return
        # _run_move drives the same CAN bus from a background thread during
        # APPROACHING/MOVING/RESETTING; polling here concurrently can steal
        # its response frame and leave that thread blocked forever on a CAN
        # read with no timeout (bus.read() -> receive() has none). _run_move
        # already publishes fresh FK after every move, so skip while busy.
        if self._fsm.state != "IDLE":
            return
        try:
            ok, x, y, z = self._ctrl.get_current_xyz()   # platform mm
            if ok:
                self._send_fk(x, y, z)
                # /delta/ee_position_mm — EE-tip coordinates in mm.
                # MATLAB's predict_target block subscribes here; without this
                # publisher the block holds zero forever in bridge mode.
                pt = PointStamped()
                pt.header.stamp    = self.get_clock().now().to_msg()
                pt.header.frame_id = "robot_base"
                pt.point.x = float(x)
                pt.point.y = float(y)
                pt.point.z = float(z) - config.EE_OFFSET_Z_MM   # platform → EE-tip
                self._pub_ee_pos_mm.publish(pt)
        except Exception as exc:
            self.get_logger().debug(f"FK heartbeat CAN error: {exc}")

    def destroy_node(self) -> None:
        if config.ENABLE_MOTORS and self._explicit_shutdown:
            try:
                self._ctrl.move_thetas(0.0, 0.0, 0.0)   # exact θ=0,0,0 home
                time.sleep(0.5)
            except Exception:
                pass
            self._ctrl.shutdown()
        super().destroy_node()


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    import signal
    rclpy.init(args=args)
    node = MatlabBridgeNode()

    def handle_sigint(*_):
        node.get_logger().info("Ctrl+C — shutting down cleanly")
        node._explicit_shutdown = True
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


if __name__ == "__main__":
    main()
