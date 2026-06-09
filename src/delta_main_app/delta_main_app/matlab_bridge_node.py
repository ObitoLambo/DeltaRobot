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
  IDLE → WAITING_MATLAB → MOVING → RESETTING → IDLE
                        ↘ ERROR  ─────────────────↗
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from custom_messages.msg import DeltaTarget, DeltaJointAngles
from delta_common import config
from delta_motor_controller.motor_controller import DeltaMotorController

MATLAB_TIMEOUT_S = 3.0   # seconds to wait for MATLAB joint-angle reply
HOME_X = config.HOME_X
HOME_Y = config.HOME_Y
HOME_Z = config.HOME_Z


# ── State machine ──────────────────────────────────────────────────────────────

class MatlabBridgeFSM:
    """
    Pure FSM — no ROS dependency; driven by the ROS node via public methods.

    States
    ------
    IDLE            : waiting for a target
    WAITING_MATLAB  : DeltaTarget published; waiting for DeltaJointAngles back
    MOVING          : executing the MATLAB theta solution on the motors (bg thread)
    RESETTING       : returning to home position after a move (bg thread)
    ERROR           : IK invalid or MATLAB timeout; recovering to IDLE (bg thread)
    """

    def __init__(self, controller: DeltaMotorController, publish_state_fn, publish_fk_fn, logger):
        self._ctrl       = controller
        self._pub_state  = publish_state_fn   # fn(state: str)
        self._pub_fk     = publish_fk_fn      # fn(x, y, z)
        self._log        = logger

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
            self._set_state("MOVING")
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

        ok, fk_xyz, err = self._ctrl.move_thetas(t1, t2, t3)
        if fk_xyz is not None:
            self._pub_fk(fk_xyz[0], fk_xyz[1], fk_xyz[2])

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
        self._ctrl.move_xyz(HOME_X, HOME_Y, HOME_Z, raw=True)
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
            self._ctrl.move_xyz(HOME_X, HOME_Y, HOME_Z, raw=True)
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

        # ── FSM ───────────────────────────────────────────────────────────────
        self._fsm = MatlabBridgeFSM(
            controller=self._ctrl,
            publish_state_fn=self._send_state,
            publish_fk_fn=self._send_fk,
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
            1,
        )
        self._sub_cam_info = self.create_subscription(
            CameraInfo,
            config.CAMERA_INFO_TOPIC,
            self._on_camera_info,
            1,
        )

        # ── 10 Hz tick for MATLAB timeout ─────────────────────────────────────
        self.create_timer(0.1, self._fsm.tick)

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

        if not self._fsm.on_target(x, y, z):
            self.get_logger().warn(
                f"Target ({x * 1000.0:.1f},{y * 1000.0:.1f},{z * 1000.0:.1f}) mm "
                f"dropped — robot busy (state={self._fsm.state})"
            )
            return

        self.get_logger().info(
            f"Target: ({x * 1000.0:.1f},{y * 1000.0:.1f},{z * 1000.0:.1f}) mm "
            f"— forwarding to MATLAB"
        )

        out = DeltaTarget()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = "robot_base"
        out.x_mm            = float(x) * 1000.0
        out.y_mm            = float(y) * 1000.0
        out.z_mm            = float(z) * 1000.0
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

    def _reproject_with_real_depth(self, x_m: float, y_m: float, z_m: float):
        """Replace z using real RealSense depth.

        Takes base-frame position in metres from /delta/target_xyz, reverse-
        projects to image pixel, reads aligned depth at that pixel, then
        re-deprojects to base frame with the real depth.

        Returns (x_m, y_m, z_real_m) or None if depth is unavailable.
        """
        if self._fx is None:
            return None

        # metres → mm
        p_b = np.array([x_m * 1000.0, y_m * 1000.0, z_m * 1000.0, 1.0])

        # Base frame → camera frame → pixel
        p_c = self._T_base_to_cam @ p_b
        xc, yc, zc = p_c[:3]
        if zc <= 1.0:
            return None
        u = int(round(self._fx * xc / zc + self._cx))
        v = int(round(self._fy * yc / zc + self._cy))

        # Sample 5×5 window from the aligned depth image
        with self._depth_lock:
            depth_img = self._depth_image
        if depth_img is None:
            return None

        h, w = depth_img.shape
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(
                f"Projected pixel ({u},{v}) outside depth image ({w}×{h})"
            )
            return None

        r = 2
        patch = depth_img[
            max(0, v - r):min(h, v + r + 1),
            max(0, u - r):min(w, u + r + 1),
        ].flatten().astype(np.float32)
        valid = patch[(patch > 1.0) & (patch < 10000.0)]   # 1 mm … 10 m
        if len(valid) < 3:
            return None

        z_cam_mm = float(np.median(valid))
        x_cam_mm = (u - self._cx) * z_cam_mm / self._fx
        y_cam_mm = (v - self._cy) * z_cam_mm / self._fy

        p_real = self._T_cam_to_base @ np.array([x_cam_mm, y_cam_mm, z_cam_mm, 1.0])
        return p_real[0] / 1000.0, p_real[1] / 1000.0, p_real[2] / 1000.0

    # ── publish helpers ───────────────────────────────────────────────────────

    def _send_state(self, state: str) -> None:
        m = String()
        m.data = state
        self._pub_state.publish(m)

    def _send_fk(self, x: float, y: float, z: float) -> None:
        pt = PointStamped()
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.header.frame_id = "robot_base"
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)
        self._pub_fk.publish(pt)

    def destroy_node(self) -> None:
        if config.ENABLE_MOTORS:
            try:
                self._ctrl.move_xyz(HOME_X, HOME_Y, HOME_Z, raw=True)
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
