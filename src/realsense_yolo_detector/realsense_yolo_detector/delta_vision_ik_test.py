#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import signal
import time
from typing import Optional
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO

from fk_ik import delta_calcInverse, delta_calcForward, e, f, re, rf


# =========================================================
# CONFIG
# =========================================================

# Camera topics
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"

# YOLO
YOLO_MODEL = "yolov8n.pt"
CONF_THRES = 0.50
TARGET_CLASS = None       # e.g. "bottle"; None = accept all
VIEW_IMAGE = True

# Workspace limits
X_LIMIT = 151.563
Y_LIMIT = 151.563
Z_MIN = -500.0
Z_MAX = -196.875

# Joint limits
#THETA1_MIN = -59.86
THETA1_MIN = 0
THETA1_MAX = 82.11

##THETA2_MIN = -68.85
THETA2_MIN = 0
THETA2_MAX = 85.49

#THETA3_MIN = -68.85
THETA3_MIN = 0
THETA3_MAX = 85.49

# Depth range
DEPTH_MIN_M = 0.001
DEPTH_MAX_M = 1.00

# Verification tolerance
FK_VERIFY_TOL_MM = 3.0

# Averaging / filtering
AVG_FRAME_COUNT = 5
CENTER_WINDOW = 5          # 7x7 median depth window
STABLE_THRESH_X_MM = 2.0
STABLE_THRESH_Y_MM = 2.0
STABLE_THRESH_Z_MM = 3.0

# Timing
PRINT_COOLDOWN_SEC = 1.0
FAKE_MOVE_COOLDOWN_SEC = 1.0

# Camera mounted at base center
CAM_TX_MM = 0.0
CAM_TY_MM = 0.0
CAM_TZ_MM = 0.0

# Camera-to-base transform mode
# Try "A" first
# A: x_base =  y_cam, y_base = -x_cam, z_base = -z_cam
# B: x_base = -y_cam, y_base =  x_cam, z_base = -z_cam
# C: x_base = -x_cam, y_base = -y_cam, z_base = -z_cam
# D: x_base =  x_cam, y_base =  y_cam, z_base = -z_cam
CAMERA_TRANSFORM_MODE = "A"

_running = True


# =========================================================
# UTILS
# =========================================================

def check_workspace(x, y, z):
    return (abs(x) <= X_LIMIT) and (abs(y) <= Y_LIMIT) and (Z_MIN <= z <= Z_MAX)


def within_joint_limits(t1, t2, t3):
    return (
        THETA1_MIN <= t1 <= THETA1_MAX and
        THETA2_MIN <= t2 <= THETA2_MAX and
        THETA3_MIN <= t3 <= THETA3_MAX
    )


def sig_handler(sig, frame):
    global _running
    _running = False


# =========================================================
# MAIN NODE
# =========================================================

class DeltaVisionIKFakeMotor(Node):
    def __init__(self):
        super().__init__("delta_vision_ik_fake_motor")

        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL)
        self.latest_depth_msg: Optional[Image] = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.last_print_time = 0.0
        self.last_fake_move_time = 0.0

        self.xyz_buffer = deque(maxlen=AVG_FRAME_COUNT)
        self.last_track_id = None
        self.last_stable_xyz = None

        self.sub_color = self.create_subscription(Image, COLOR_TOPIC, self.color_callback, 10)
        self.sub_depth = self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.info_callback, 10)

        self.get_logger().info("✅ Delta vision IK + fake motor node ready")

        if VIEW_IMAGE:
            cv2.namedWindow("Delta Vision + IK + Fake Motor", cv2.WINDOW_NORMAL)

    def info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Image):
        self.latest_depth_msg = msg

    def refine_center_from_roi(self, frame, x1, y1, x2, y2):
        """
        Refine center using contour centroid from shrunken ROI.
        Fallback = bbox center.
        """
        w = x2 - x1
        h = y2 - y1

        sx1 = int(x1 + 0.2 * w)
        sy1 = int(y1 + 0.2 * h)
        sx2 = int(x2 - 0.2 * w)
        sy2 = int(y2 - 0.2 * h)

        if sx2 <= sx1 or sy2 <= sy1:
            return int((x1 + x2) / 2), int((y1 + y2) / 2)

        roi = frame[sy1:sy2, sx1:sx2]
        if roi.size == 0:
            return int((x1 + x2) / 2), int((y1 + y2) / 2)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        contours1, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours2, _ = cv2.findContours(255 - th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_area = 0.0

        for contours in (contours1, contours2):
            for c in contours:
                area = cv2.contourArea(c)
                if area > best_area:
                    best_area = area
                    best_contour = c

        if best_contour is None or best_area < 30:
            return int((x1 + x2) / 2), int((y1 + y2) / 2)

        M = cv2.moments(best_contour)
        if abs(M["m00"]) < 1e-6:
            return int((x1 + x2) / 2), int((y1 + y2) / 2)

        cu = int(M["m10"] / M["m00"]) + sx1
        cv = int(M["m01"] / M["m00"]) + sy1
        return cu, cv

    def get_depth_meters(self, u: int, v: int) -> Optional[float]:
        if self.latest_depth_msg is None:
            return None

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                self.latest_depth_msg,
                desired_encoding="passthrough"
            )
        except Exception as ex:
            self.get_logger().warning(f"Depth conversion failed: {ex}")
            return None

        h, w = depth_image.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None

        win = CENTER_WINDOW
        x1 = max(0, u - win)
        x2 = min(w, u + win + 1)
        y1 = max(0, v - win)
        y2 = min(h, v + win + 1)
        roi = depth_image[y1:y2, x1:x2]

        if depth_image.dtype == np.uint16:
            valid = roi[roi > 0]
            if valid.size == 0:
                return None
            z = float(np.median(valid)) * 0.001

        elif depth_image.dtype in (np.float32, np.float64):
            valid = roi[np.isfinite(roi) & (roi > 0)]
            if valid.size == 0:
                return None
            z = float(np.median(valid))
        else:
            return None

        if z < DEPTH_MIN_M or z > DEPTH_MAX_M:
            return None

        return z

    def pixel_to_camera_xyz_mm(self, u: int, v: int, z_m: float):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None

        z_mm = z_m * 1000.0
        x_mm = ((u - self.cx) * z_mm) / self.fx
        y_mm = ((v - self.cy) * z_mm) / self.fy
        return x_mm, y_mm, z_mm

    def camera_to_base_mm(self, x_cam: float, y_cam: float, z_cam: float):
        mode = CAMERA_TRANSFORM_MODE.upper()

        if mode == "A":
            x_base = y_cam + CAM_TX_MM
            y_base = -x_cam + CAM_TY_MM
            z_base = -z_cam + CAM_TZ_MM
        elif mode == "B":
            x_base = -y_cam + CAM_TX_MM
            y_base = x_cam + CAM_TY_MM
            z_base = -z_cam + CAM_TZ_MM
        elif mode == "C":
            x_base = -x_cam + CAM_TX_MM
            y_base = -y_cam + CAM_TY_MM
            z_base = -z_cam + CAM_TZ_MM
        else:  # D
            x_base = x_cam + CAM_TX_MM
            y_base = y_cam + CAM_TY_MM
            z_base = -z_cam + CAM_TZ_MM

        return x_base, y_base, z_base

    def update_xyz_average(self, track_id: str, xyz_now):
        if self.last_track_id != track_id:
            self.xyz_buffer.clear()
            self.last_stable_xyz = None
            self.last_track_id = track_id

        self.xyz_buffer.append(xyz_now)

        if len(self.xyz_buffer) < AVG_FRAME_COUNT:
            return None

        arr = np.array(self.xyz_buffer, dtype=np.float64)
        xyz_avg = np.mean(arr, axis=0)
        return float(xyz_avg[0]), float(xyz_avg[1]), float(xyz_avg[2])

    def is_stable(self, xyz_now):
        if self.last_stable_xyz is None:
            self.last_stable_xyz = xyz_now
            return False

        dx = abs(xyz_now[0] - self.last_stable_xyz[0])
        dy = abs(xyz_now[1] - self.last_stable_xyz[1])
        dz = abs(xyz_now[2] - self.last_stable_xyz[2])

        self.last_stable_xyz = xyz_now

        return (
            dx <= STABLE_THRESH_X_MM and
            dy <= STABLE_THRESH_Y_MM and
            dz <= STABLE_THRESH_Z_MM
        )

    def validate_target(self, x_base, y_base, z_base):
        if not check_workspace(x_base, y_base, z_base):
            return None, None, None, False, "OUTSIDE_WORKSPACE"

        st_ik, t1, t2, t3 = delta_calcInverse(x_base, y_base, z_base, e, f, re, rf)
        if st_ik != 0:
            return None, None, None, False, "IK_FAILED"

        if not within_joint_limits(t1, t2, t3):
            return (t1, t2, t3), None, None, False, "JOINT_LIMIT"

        st_fk, x_fk, y_fk, z_fk = delta_calcForward(t1, t2, t3, e, f, re, rf)
        if st_fk != 0:
            return (t1, t2, t3), None, None, False, "FK_FAILED"

        fk_xyz = (x_fk, y_fk, z_fk)
        fk_err = math.sqrt(
            (x_base - x_fk) ** 2 +
            (y_base - y_fk) ** 2 +
            (z_base - z_fk) ** 2
        )

        if fk_err > FK_VERIFY_TOL_MM:
            return (t1, t2, t3), fk_xyz, fk_err, False, "FK_MISMATCH"

        return (t1, t2, t3), fk_xyz, fk_err, True, "OK"

    def maybe_print_result(self, best):
        now = time.time()
        if now - self.last_print_time < PRINT_COOLDOWN_SEC:
            return

        print("\n================ RESULT ================")
        print(f"name         : {best['name']}")
        print(f"conf         : {best['conf']:.2f}")
        print(f"center_uv    : ({best['u']}, {best['v']})")
        print(f"raw_cam_xyz  : ({best['x_cam']:.2f}, {best['y_cam']:.2f}, {best['z_cam']:.2f}) mm")
        print(f"avg_base_xyz : ({best['x_base']:.2f}, {best['y_base']:.2f}, {best['z_base']:.2f}) mm")
        print(f"in_workspace : {check_workspace(best['x_base'], best['y_base'], best['z_base'])}")

        if best["ik_deg"] is not None:
            print(f"theta_deg    : ({best['ik_deg'][0]:.2f}, {best['ik_deg'][1]:.2f}, {best['ik_deg'][2]:.2f})")
        else:
            print("theta_deg    : None")

        if best["fk_xyz"] is not None:
            print(f"fk_xyz       : ({best['fk_xyz'][0]:.2f}, {best['fk_xyz'][1]:.2f}, {best['fk_xyz'][2]:.2f}) mm")
        else:
            print("fk_xyz       : None")

        if best["fk_err"] is not None:
            print(f"fk_error     : {best['fk_err']:.2f} mm")
        else:
            print("fk_error     : None")

        print(f"move_allow   : {best['allowed']}")
        print(f"status       : {best['reason']}")
        print("========================================\n")

        self.last_print_time = now

    def fake_motor_command(self, best):
        now = time.time()
        if now - self.last_fake_move_time < FAKE_MOVE_COOLDOWN_SEC:
            return

        if not best["allowed"] or best["ik_deg"] is None:
            return

        t1, t2, t3 = best["ik_deg"]

        print("----------- FAKE MOTOR COMMAND -----------")
        print(f"FAKE MOVE XYZ   : ({best['x_base']:.2f}, {best['y_base']:.2f}, {best['z_base']:.2f}) mm")
        print(f"FAKE MOVE THETA : ({t1:.2f}, {t2:.2f}, {t3:.2f}) deg")

        if best["fk_xyz"] is not None:
            print(f"FAKE FK XYZ     : ({best['fk_xyz'][0]:.2f}, {best['fk_xyz'][1]:.2f}, {best['fk_xyz'][2]:.2f}) mm")

        if best["fk_err"] is not None:
            print(f"FAKE FK ERR     : {best['fk_err']:.2f} mm")

        print("------------------------------------------")

        self.last_fake_move_time = now

    def color_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as ex:
            self.get_logger().error(f"Color conversion failed: {ex}")
            return

        results = self.model.predict(
            source=frame,
            conf=CONF_THRES,
            verbose=False,
            device="cpu"
        )

        annotated = frame.copy()
        best = None

        if len(results) > 0:
            result = results[0]
            boxes = result.boxes

            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int).tolist()
                    conf = float(box.conf[0].item())
                    cls_id = int(box.cls[0].item())
                    name = self.model.names.get(cls_id, str(cls_id))

                    if TARGET_CLASS is not None and name != TARGET_CLASS:
                        continue

                    u, v = self.refine_center_from_roi(frame, x1, y1, x2, y2)

                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(annotated, (u, v), 5, (0, 0, 255), -1)

                    z_m = self.get_depth_meters(u, v)
                    if z_m is None:
                        label = f"{name} {conf:.2f} C=({u},{v}) no-depth"
                        cv2.putText(
                            annotated, label, (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 2, cv2.LINE_AA
                        )
                        continue

                    xyz_cam = self.pixel_to_camera_xyz_mm(u, v, z_m)
                    if xyz_cam is None:
                        continue

                    x_cam, y_cam, z_cam = xyz_cam
                    x_base_now, y_base_now, z_base_now = self.camera_to_base_mm(x_cam, y_cam, z_cam)

                    bbox_cu = int((x1 + x2) / 2)
                    bbox_cv = int((y1 + y2) / 2)
                    track_id = f"{name}_{bbox_cu}_{bbox_cv}"

                    xyz_avg = self.update_xyz_average(track_id, (x_base_now, y_base_now, z_base_now))

                    ik_deg = None
                    fk_xyz = None
                    fk_err = None
                    allowed = False
                    reason = "WAIT_AVG"

                    x_use, y_use, z_use = x_base_now, y_base_now, z_base_now

                    if xyz_avg is not None:
                        x_use, y_use, z_use = xyz_avg

                        if not self.is_stable(xyz_avg):
                            reason = "NOT_STABLE"
                        else:
                            ik_deg, fk_xyz, fk_err, allowed, reason = self.validate_target(
                                x_use, y_use, z_use
                            )

                    theta_text = ""
                    if ik_deg is not None:
                        theta_text = f"T=({ik_deg[0]:.1f},{ik_deg[1]:.1f},{ik_deg[2]:.1f}) "

                    color = (0, 255, 0) if allowed else (0, 0, 255)
                    label = (
                        f"{name} {conf:.2f} "
                        f"C=({u},{v}) "
                        f"XYZ=({x_use:.1f},{y_use:.1f},{z_use:.1f}) "
                        f"{theta_text}{reason}"
                    )
                    cv2.putText(
                        annotated, label, (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        color, 2, cv2.LINE_AA
                    )

                    cand = {
                        "name": name,
                        "conf": conf,
                        "u": u,
                        "v": v,
                        "x_cam": x_cam,
                        "y_cam": y_cam,
                        "z_cam": z_cam,
                        "x_base": x_use,
                        "y_base": y_use,
                        "z_base": z_use,
                        "ik_deg": ik_deg,
                        "fk_xyz": fk_xyz,
                        "fk_err": fk_err,
                        "allowed": allowed,
                        "reason": reason
                    }

                    if best is None or z_use < best["z_base"]:
                        best = cand

        if best is not None:
            self.maybe_print_result(best)
            self.fake_motor_command(best)

        if VIEW_IMAGE:
            cv2.imshow("Delta Vision + IK + Fake Motor", annotated)
            cv2.waitKey(1)

    def destroy_node(self):
        if VIEW_IMAGE:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    global _running
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    rclpy.init(args=args)
    node = DeltaVisionIKFakeMotor()

    try:
        while rclpy.ok() and _running:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()