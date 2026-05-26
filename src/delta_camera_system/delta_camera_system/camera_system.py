#!/usr/bin/env python3

# CHANGES: [1.1] depth lock and stale-depth protection, [1.2] reusable CLAHE/kernel allocation, [2.1] config validation, [2.3] stable anchor fix, [2.5] target/status publishers, [2.6] centralized tracking reset

import math
import threading
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ultralytics import YOLO

from delta_common import config
from delta_common.fk_ik import delta_calcForward, delta_calcInverse, e, f, re, rf


class DeltaCamera(Node):
    def __init__(self):
        super().__init__("delta_camera_system")

        self.bridge = CvBridge()
        self.DETECTION_MODE = config.DETECTION_MODE.lower()
        self.model = YOLO(config.YOLO_MODEL) if self.DETECTION_MODE == "yolo" else None
        self.latest_depth_msg: Optional[Image] = None
        self._depth_lock = threading.Lock()
        self.latest_depth_stamp = None
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self._kernel_5 = np.ones((5, 5), dtype=np.uint8)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_matrix = None
        self.dist_coeffs = None

        self.CONF_THRES = config.CONF_THRES
        self.TARGET_CLASS = config.TARGET_CLASS
        self.VIEW_IMAGE = config.VIEW_IMAGE
        self.YOLO_IMGSZ = config.YOLO_IMGSZ
        self.YOLO_IOU = config.YOLO_IOU
        self.YOLO_MAX_DET = config.YOLO_MAX_DET
        self.YOLO_DEVICE = config.YOLO_DEVICE

        self.last_print_time = 0.0
        self.last_fake_move_time = 0.0
        self._last_target_publish_time = 0.0
        self._confirm_counts: dict = {}
        self.no_detection_count = 0
        self._frame_times: deque = deque(maxlen=30)
        self._proc_times: deque = deque(maxlen=30)

        self.T_cam_to_base, self.T_base_to_cam = self._build_T_cam_to_base()
        self.camera_rotation    = self.T_cam_to_base[:3, :3]
        self.camera_translation = self.T_cam_to_base[:3,  3]
        self.plane_homography = self.build_plane_homography()

        self._xyz_buffers: dict = {}        # per-track xyz history
        self._timed_xyz_bufs: dict = {}     # per-track timestamped xyz history
        self.depth_buffer = deque(maxlen=config.DEPTH_HISTORY_SIZE)
        self.last_depth_track_id = None
        self.last_stable_xyz = None

        self.sub_color = self.create_subscription(
            Image, config.COLOR_TOPIC, self.color_callback, 10
        )
        self.sub_depth = self.create_subscription(
            Image, config.DEPTH_TOPIC, self.depth_callback, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, config.CAMERA_INFO_TOPIC, self.info_callback, 10
        )
        self._pub_target      = self.create_publisher(PointStamped, "/delta/target_xyz",          10)
        self._pub_velocity    = self.create_publisher(PointStamped, "/delta/object_velocity_mm_s", 10)
        self._pub_status      = self.create_publisher(String,       "/delta/detection_status",     10)
        self._pub_all_targets = self.create_publisher(PoseArray,    "/delta/all_targets",          10)
        self._pub_ee_pos      = self.create_publisher(PointStamped, "/delta/ee_position_mm",       10)
        self._pub_ee_error    = self.create_publisher(PointStamped, "/delta/ee_error_mm",          10)
        self.create_service(Trigger, "/delta/calibrate_cam_offset", self._calibrate_offset_srv)

        mode = "FAKE DEPTH" if config.FAKE_DEPTH_ENABLE else "REAL DEPTH"
        self.get_logger().info(
            f"Delta camera system ready ({mode}, detection={self.DETECTION_MODE})"
        )

        if self.VIEW_IMAGE:
            cv2.namedWindow("Delta Camera", cv2.WINDOW_NORMAL)

        self._cal_active = False
        self._cal_samples: list = []

        self.reset_tracking_state()
        self._validate_config()

    def info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_matrix = np.array(
            ((self.fx, 0.0, self.cx), (0.0, self.fy, self.cy), (0.0, 0.0, 1.0)),
            dtype=np.float64,
        )
        if len(msg.d) > 0:
            self.dist_coeffs = np.asarray(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def depth_callback(self, msg: Image):
        with self._depth_lock:
            self.latest_depth_msg = msg
            self.latest_depth_stamp = self.get_clock().now()

    def color_callback(self, msg: Image):
        if self.fx is None:
            return

        t_start = time.perf_counter()
        self._frame_times.append(t_start)

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as ex:
            self.get_logger().error(f"Color conversion failed: {ex}")
            return

        annotated, best = self.process_frame(frame)

        t_end = time.perf_counter()
        self._proc_times.append(t_end - t_start)

        if best is not None:
            self.maybe_print_result(best)
            self.fake_motor_command(best)
            self._collect_cal_sample(best)

        if self.VIEW_IMAGE:
            self._draw_perf_overlay(annotated)
            cv2.imshow("Delta Camera", annotated)
            cv2.waitKey(1)

    def _draw_perf_overlay(self, frame):
        if len(self._frame_times) >= 2:
            dt = self._frame_times[-1] - self._frame_times[0]
            fps = (len(self._frame_times) - 1) / dt if dt > 0 else 0.0
        else:
            fps = 0.0
        avg_ms = (sum(self._proc_times) / len(self._proc_times) * 1000) if self._proc_times else 0.0
        text = f"FPS: {fps:.1f}  proc: {avg_ms:.1f} ms"
        cv2.putText(frame, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

    def process_frame(self, frame):
        annotated = frame.copy()
        best = None
        pure_camera_mode = bool(config.PURE_CAMERA_TEST_ENABLE)
        depth_image = self.get_depth_image()
        frame_h, frame_w = frame.shape[:2]
        self.draw_workspace_overlay(annotated, frame_w, frame_h)
        if config.DRAW_CAMERA_AXIS_LEGEND:
            self.draw_camera_axis_legend(annotated)

        # ── EE marker detection ───────────────────────────────────────────────
        ee_pixel = None
        ee_base_xy = None
        ee_uv = self.detect_ee_marker(frame)
        if ee_uv is not None:
            eu, ev = ee_uv
            ee_z_m = None
            if depth_image is not None:
                r = 8  # sample within 8px of EE centroid
                vals = self.collect_depth_values_meters(
                    depth_image, eu, ev, eu - r, ev - r, eu + r, ev + r)
                z_robust = self.robust_depth_from_values(vals)
                if z_robust is not None:
                    ee_z_m = z_robust
            if ee_z_m is None:
                ee_z_m = config.FAKE_DEPTH_M
            ee_cam = self.pixel_to_camera_xyz_mm(eu, ev, ee_z_m)
            if ee_cam is not None:
                ex_b, ey_b, ez_b = self.camera_to_base_mm(*ee_cam)
                ee_pixel = (eu, ev)
                ee_base_xy = (ex_b, ey_b)
                pt = PointStamped()
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.header.frame_id = "robot_base"
                pt.point.x = ex_b
                pt.point.y = ey_b
                pt.point.z = ez_b
                self._pub_ee_pos.publish(pt)
            if getattr(config, "DRAW_EE_MARKER", True):
                cv2.circle(annotated, (eu, ev), 6, (255, 255, 0), 2)
                cv2.putText(annotated, "EE", (eu + 8, ev),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 0), 1)

        detections = self.generate_detections(frame, depth_image)
        if not detections:
            self.handle_no_detection()
            return annotated, best

        self.no_detection_count = 0
        all_candidates = []

        for det in detections:
            x1 = det["x1"]
            y1 = det["y1"]
            x2 = det["x2"]
            y2 = det["y2"]
            conf = det["conf"]
            name = det["name"]
            bbox_center = det.get("bbox_center", (int((x1 + x2) / 2), int((y1 + y2) / 2)))
            contour = det.get("primary_contour")
            size_contour = det.get("outer_contour")
            corners_uv = det.get("corners_uv")
            depth_contour = det.get("depth_contour")
            depth_bbox = det.get("depth_bbox")

            if config.REQUIRE_FULL_BBOX_IN_FRAME and not self.bbox_inside_frame(frame_w, frame_h, x1, y1, x2, y2):
                if config.DRAW_REJECTED_DETECTIONS:
                    self.draw_rejected_detection(
                        annotated,
                        x1,
                        y1,
                        x2,
                        y2,
                        f"{name} {conf:.2f} OUT_OF_FRAME",
                    )
                continue

            if contour is None:
                bbox_center, contour = self.extract_primary_contour(frame, x1, y1, x2, y2)

            require_box_like = config.BOX_ONLY_ENABLE and not det.get("validated_box_like", False)
            if self.DETECTION_MODE == "depth_foreground" and not config.FOREGROUND_REQUIRE_BOX_LIKE:
                require_box_like = False

            if require_box_like and not self.is_box_like(contour):
                if config.DRAW_REJECTED_DETECTIONS:
                    self.draw_rejected_detection(
                        annotated,
                        x1,
                        y1,
                        x2,
                        y2,
                        f"{name} {conf:.2f} NOT_BOX",
                    )
                continue

            if size_contour is None:
                size_contour = contour if contour is not None else self.extract_outer_contour(frame, x1, y1, x2, y2)

            center_contour = size_contour if size_contour is not None else contour
            if corners_uv is not None:
                u = int(round(float(np.mean(corners_uv[:, 0]))))
                v = int(round(float(np.mean(corners_uv[:, 1]))))
                center_contour = np.round(corners_uv).astype(np.int32).reshape(-1, 1, 2)
            else:
                u, v = self.contour_center_or_bbox(bbox_center, center_contour)
            track_id = self.make_track_id(name, x1, y1, x2, y2)
            track_confirmed = self.update_detection_confirmation(track_id)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(annotated, (u, v), 5, (0, 0, 255), -1)
            if center_contour is not None and config.DRAW_DETECTION_CONTOUR:
                cv2.drawContours(annotated, [center_contour], -1, (255, 255, 0), 2)
            if corners_uv is not None and config.DRAW_DETECTION_CORNERS:
                for corner in np.round(corners_uv).astype(np.int32):
                    cv2.circle(annotated, tuple(corner.tolist()), 4, (0, 255, 255), -1)

            depth_x1, depth_y1, depth_x2, depth_y2 = (
                depth_bbox if depth_bbox is not None else (x1, y1, x2, y2)
            )

            z_m = self.get_depth_meters(
                depth_image,
                u,
                v,
                depth_x1,
                depth_y1,
                depth_x2,
                depth_y2,
                track_id,
                contour=depth_contour if depth_contour is not None else center_contour,
            )
            depth_is_approx = False
            if z_m is None:
                if config.FAKE_DEPTH_ENABLE:
                    z_m = config.FAKE_DEPTH_M
                    depth_is_approx = True
                else:
                    label = f"{name} {conf:.2f} C=({u},{v}) NO_DEPTH"
                    cv2.putText(
                        annotated,
                        label,
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    continue

            xyz_cam = self.pixel_to_camera_xyz_mm(u, v, z_m)
            if xyz_cam is None:
                continue

            x_cam, y_cam, z_cam = xyz_cam
            x_cam_parallel, y_cam_parallel, z_cam_parallel = self.camera_to_parallel_mm(
                x_cam, y_cam, z_cam
            )
            plane_center_xy, plane_yaw_deg, plane_corners_xy = self.estimate_plane_center_and_yaw(corners_uv) if corners_uv is not None else (None, None, None)
            pose_info = self.estimate_rectangle_pose(corners_uv) if corners_uv is not None else None
            size_info = self.estimate_object_size(size_contour, x1, y1, x2, y2, z_cam)
            x_base_tf, y_base_tf, z_base_now = self.camera_to_base_mm(x_cam, y_cam, z_cam)
            if pure_camera_mode:
                x_base_now, y_base_now = x_base_tf, y_base_tf
                xy_source = "camera_only"
            elif plane_center_xy is not None:
                x_base_now, y_base_now = plane_center_xy
                xy_source = "homography"
            else:
                x_base_now, y_base_now = x_base_tf, y_base_tf
                xy_source = "camera_transform"
            xyz_avg = self.update_xyz_average(track_id, (x_base_now, y_base_now, z_base_now))

            workspace_xyz = xyz_avg if xyz_avg is not None else (x_base_now, y_base_now, z_base_now)
            wx, wy, wz = workspace_xyz
            in_workspace = self.check_workspace(wx, wy, wz)
            # In conveyor mode, also allow objects in the approach zone (y > Y_LIMIT)
            # so the robot can pre-position while the object is still incoming.
            in_approach_zone = (
                config.CONVEYOR_MODE
                and abs(wx) <= config.X_LIMIT
                and wy > config.Y_LIMIT
                and config.Z_MIN <= wz <= config.Z_MAX
                and len(self._timed_xyz_bufs.get(track_id, [])) >= 3
            )
            if config.DETECT_ONLY_IN_WORKSPACE and not in_workspace and not in_approach_zone:
                if config.DRAW_REJECTED_DETECTIONS:
                    self.draw_rejected_detection(
                        annotated,
                        x1,
                        y1,
                        x2,
                        y2,
                        f"{name} {conf:.2f} OUTSIDE_WORKSPACE",
                    )
                continue

            ik_deg = None
            fk_xyz = None
            fk_err = None
            allowed = False
            reason = "WAIT_TRACK"
            x_use, y_use, z_use = x_base_now, y_base_now, z_base_now

            if pure_camera_mode:
                allowed = True
                reason = "CAMERA_OK"
            elif track_confirmed:
                reason = "WAIT_AVG"
                if xyz_avg is not None:
                    x_use, y_use, z_use = xyz_avg
                    if config.CONVEYOR_MODE:
                        # Object is moving — skip stability check, use averaged position.
                        # Also allow early trigger from approach zone.
                        allowed = True
                        reason = "CONVEYOR_OK" if in_workspace else "APPROACH_ZONE"
                    elif not self.is_stable(xyz_avg):
                        reason = "NOT_STABLE"
                    elif config.VISION_ONLY_ENABLE:
                        allowed = True
                        reason = "OK"
                    else:
                        ik_deg, fk_xyz, fk_err, allowed, reason = self.validate_target(
                            x_use, y_use, z_use
                        )

            theta_text = ""
            if ik_deg is not None:
                theta_text = f"T=({ik_deg[0]:.1f},{ik_deg[1]:.1f},{ik_deg[2]:.1f}) "

            pose_yaw = None if pose_info is None else pose_info["pose_yaw_deg"]
            display_yaw = pose_yaw if pose_yaw is not None else plane_yaw_deg

            if depth_is_approx:
                label_color = (0, 255, 255)   # yellow — approximate depth
            elif allowed:
                label_color = (0, 255, 0)
            else:
                label_color = (0, 0, 255)
            if pure_camera_mode:
                label = f"{name} {conf:.2f} CAM=({x_cam:.1f},{y_cam:.1f},{z_cam:.1f}) {reason}"
            elif config.VISION_ONLY_ENABLE:
                label = (
                    f"{name} {conf:.2f} "
                    f"XYZ=({x_use:.1f},{y_use:.1f},{z_use:.1f}) "
                )
                if display_yaw is not None:
                    label += f"YAW={display_yaw:.1f} "
                label += reason
                if depth_is_approx:
                    label += " APPROX"
            else:
                label = (
                    f"{name} {conf:.2f} "
                    f"C=({u},{v}) "
                    f"BASE=({x_use:.1f},{y_use:.1f},{z_use:.1f}) "
                    f"{theta_text}{reason}"
                )
                if depth_is_approx:
                    label += " APPROX"
            label_y = max(20, y1 - 8)
            cv2.putText(
                annotated,
                label,
                (x1, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                label_color,
                2,
                cv2.LINE_AA,
            )

            if config.DRAW_OBJECT_SIZE_LABEL:
                size_label = (
                    f"SIZE=({size_info['obj_size_mm'][0]:.1f}x{size_info['obj_size_mm'][1]:.1f})mm "
                    f"{size_info['size_source']}"
                )
                size_y = min(annotated.shape[0] - 8, label_y + 18)
                cv2.putText(
                    annotated,
                    size_label,
                    (x1, size_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            candidate = {
                "name": name,
                "conf": conf,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "u": u,
                "v": v,
                "x_cam": x_cam,
                "y_cam": y_cam,
                "z_cam": z_cam,
                "x_cam_parallel": x_cam_parallel,
                "y_cam_parallel": y_cam_parallel,
                "z_cam_parallel": z_cam_parallel,
                "plane_xy": plane_center_xy,
                "plane_corners_xy": plane_corners_xy,
                "xy_source": xy_source,
                "yaw_deg": plane_yaw_deg,
                "x_base": x_use,
                "y_base": y_use,
                "z_base": z_use,
                "bbox_size_px": size_info["bbox_size_px"],
                "obj_size_px": size_info["obj_size_px"],
                "obj_size_mm": size_info["obj_size_mm"],
                "size_source": size_info["size_source"],
                "pose_cam_xyz": None if pose_info is None else pose_info["pose_cam_xyz"],
                "pose_base_xyz": None if pose_info is None else pose_info["pose_base_xyz"],
                "pose_yaw_deg": display_yaw,
                "ik_deg": ik_deg,
                "fk_xyz": fk_xyz,
                "fk_err": fk_err,
                "allowed": allowed,
                "reason": reason,
                "track_id": track_id,
            }

            all_candidates.append(candidate)

            best_depth_key = z_cam if pure_camera_mode else z_use
            current_best_depth_key = None
            if best is not None:
                current_best_depth_key = best["z_cam"] if pure_camera_mode else best["z_base"]

            if best is None or best_depth_key < current_best_depth_key:
                best = candidate

        if best is not None:
            status_msg = String()
            status_msg.data = best["reason"]
            self._pub_status.publish(status_msg)
            if best["allowed"]:
                now_t = time.time()
                cooldown_ok = (now_t - self._last_target_publish_time) >= config.TARGET_PUBLISH_COOLDOWN_SEC
                if cooldown_ok:
                    self._last_target_publish_time = now_t
                    stamp = self.get_clock().now().to_msg()

                    pt = PointStamped()
                    pt.header.stamp = stamp
                    pt.header.frame_id = "robot_base"
                    pt.point.x = best["x_base"] / 1000.0
                    pt.point.z = best["z_base"] / 1000.0
                    best_timed_buf = self._timed_xyz_bufs.get(best["track_id"], deque())
                    if config.CONVEYOR_MODE and len(best_timed_buf) > 0:
                        # Use the most recent raw Y to avoid the ~(AVG_FRAME_COUNT/2)-frame
                        # median lag. The robot's prediction adds vy*t_travel on top of this.
                        pt.point.y = best_timed_buf[-1][2] / 1000.0
                    else:
                        pt.point.y = best["y_base"] / 1000.0
                    self._pub_target.publish(pt)

                    vel = PointStamped()
                    vel.header.stamp = stamp
                    vel.header.frame_id = "robot_base"
                    vel.point.y = self._estimate_vy(best_timed_buf)
                    self._pub_velocity.publish(vel)

        allowed_targets = [c for c in all_candidates if c["allowed"]]
        if allowed_targets:
            pa = PoseArray()
            pa.header.stamp = self.get_clock().now().to_msg()
            pa.header.frame_id = "robot_base"
            for c in allowed_targets:
                p = Pose()
                p.position.x = c["x_base"] / 1000.0
                p.position.y = c["y_base"] / 1000.0
                p.position.z = c["z_base"] / 1000.0
                pa.poses.append(p)
            self._pub_all_targets.publish(pa)

        # ── EE vs object error display ────────────────────────────────────────
        if (ee_pixel is not None and best is not None
                and self.fx is not None and self.fy is not None):
            ex, ey = ee_pixel
            ox, oy = best["u"], best["v"]
            du = ox - ex
            dv = oy - ey
            z_cam_mm = config.FAKE_DEPTH_M * 1000.0
            dx_mm = du * z_cam_mm / self.fx
            dy_mm = dv * z_cam_mm / self.fy
            cv2.line(annotated, (ex, ey), (ox, oy), (0, 255, 255), 1)
            mx, my = (ex + ox) // 2, (ey + oy) // 2
            cv2.putText(annotated, f"err {dx_mm:+.0f},{dy_mm:+.0f}mm",
                        (mx + 4, my), cv2.FONT_HERSHEY_SIMPLEX,
                        0.25, (0, 255, 255), 1)
            err_pt = PointStamped()
            err_pt.header.stamp = self.get_clock().now().to_msg()
            err_pt.header.frame_id = "robot_base"
            err_pt.point.x = dx_mm
            err_pt.point.y = dy_mm
            err_pt.point.z = 0.0
            self._pub_ee_error.publish(err_pt)
            self._ee_err_frame = getattr(self, "_ee_err_frame", 0) + 1
            if self._ee_err_frame % 10 == 0:
                self.get_logger().info(
                    f"EE vs obj: dx={dx_mm:+.1f} dy={dy_mm:+.1f} mm  "
                    f"(du={du} dv={dv} px)"
                )

        return annotated, best

    def generate_detections(self, frame, depth_image=None):
        if self.DETECTION_MODE == "white_rectangle":
            rect_detections = self.find_white_rectangle_detections(frame)
            return self.prioritize_workspace_detections(frame, rect_detections)
        if self.DETECTION_MODE == "blue_rectangle":
            rect_detections = self.find_blue_rectangle_detections(frame)
            return self.prioritize_workspace_detections(frame, rect_detections)
        if self.DETECTION_MODE == "orange_square":
            rect_detections = self.find_orange_square_detections(frame)
            return self.prioritize_workspace_detections(frame, rect_detections)
        if self.DETECTION_MODE == "orange_blob":
            return self.prioritize_workspace_detections(frame, self.find_orange_blob_detections(frame))
        if self.DETECTION_MODE == "depth_foreground":
            depth_detections = self.find_depth_foreground_detections(frame, depth_image)
            if depth_detections:
                return depth_detections
            bbox_detections = self.find_bbox_detections(frame)
            return self.prioritize_workspace_detections(frame, bbox_detections)
        if self.DETECTION_MODE == "bbox_only":
            return self.prioritize_workspace_detections(frame, self.find_bbox_detections(frame))
        return self.find_yolo_detections(frame)

    def draw_workspace_overlay(self, annotated, frame_w: int, frame_h: int):
        if config.DRAW_WORKSPACE_ZONES:
            self.draw_conveyor_zones(annotated, frame_w, frame_h)

        if config.WORKSPACE_ROI_ENABLE:
            x1, y1, x2, y2 = self.get_workspace_roi(frame_w, frame_h)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 200, 0), 1)
            if config.DRAW_BASE_AXIS_OVERLAY:
                self.draw_base_axis_overlay(annotated, x1, y1, x2, y2)

        if config.ROBOT_EXCLUDE_ENABLE and config.DRAW_ROBOT_EXCLUDE:
            overlay = annotated.copy()
            for polygon in self.get_robot_exclusion_polygons(frame_w, frame_h):
                cv2.fillPoly(overlay, [polygon], (40, 40, 180))
                cv2.polylines(annotated, [polygon], True, (60, 60, 255), 2)
            cv2.addWeighted(overlay, 0.18, annotated, 0.82, 0.0, annotated)

    def draw_conveyor_zones(self, annotated, frame_w: int, frame_h: int):
        """Overlay three zones projected from robot base frame onto the camera image.

        Zone layout (conveyor moves objects in the -Y direction of robot base):
          APPROACH (amber) — object visible but not yet in robot reach (y > +Y_LIMIT)
          WORKSPACE (green) — robot can pick here (|x|,|y| within ±X_LIMIT/Y_LIMIT)
          EXIT (red) — object has passed workspace (y < -Y_LIMIT)

        The workspace square is projected to pixels using T_base_to_cam + pinhole
        projection, so it automatically accounts for camera position and orientation.
        """
        if self.fx is None:
            return

        L = config.X_LIMIT          # 151.563 mm — half-width of workspace square
        z = config.WORKSPACE_PICK_Z_MM  # belt surface in base frame
        ox = getattr(config, "WORKSPACE_OVERLAY_X_OFFSET_MM", 0.0)
        oy = getattr(config, "WORKSPACE_OVERLAY_Y_OFFSET_MM", 0.0)

        # Project 4 corners of the reachable square at pick Z.
        # Split into entry edge (y=+L, where belt objects arrive) and
        # exit edge (y=-L, where objects leave robot reach).
        entry_px, exit_px = [], []
        for sx in (-1.0, 1.0):
            for sy, bucket in ((+1.0, entry_px), (-1.0, exit_px)):
                pt = self._project_base_to_pixel(sx * L + ox, sy * L + oy, z)
                if pt is None:
                    return   # camera not ready or point behind camera
                bucket.append(pt)

        # Sort each edge by pixel u so polygon vertices wind consistently
        entry_px.sort(key=lambda p: p[0])
        exit_px.sort(key=lambda p: p[0])
        el, er = entry_px   # left & right pixel of entry edge (high v, near bottom)
        xl, xr = exit_px    # left & right pixel of exit edge  (low v,  near top)

        # Skip drawing if the projection is wildly outside the frame
        all_v = [el[1], er[1], xl[1], xr[1]]
        if min(all_v) > 2 * frame_h or max(all_v) < -frame_h:
            return

        overlay = annotated.copy()

        # ── fill zones ────────────────────────────────────────────────────────
        # Approach: entry edge → bottom of image
        ap_poly = np.array([[0, frame_h], [frame_w, frame_h],
                             list(er), list(el)], dtype=np.int32)
        cv2.fillPoly(overlay, [ap_poly], (0, 160, 255))          # amber

        # Robot workspace: the 4 projected corners
        ws_poly = np.array([list(el), list(er), list(xr), list(xl)], dtype=np.int32)
        self._ws_poly = ws_poly   # cached for EE bounds filter
        cv2.fillPoly(overlay, [ws_poly], (0, 200, 60))           # green

        # Exit: top of image → exit edge
        ex_poly = np.array([[0, 0], [frame_w, 0],
                             list(xr), list(xl)], dtype=np.int32)
        cv2.fillPoly(overlay, [ex_poly], (60, 60, 200))          # red

        cv2.addWeighted(overlay, 0.20, annotated, 0.80, 0.0, annotated)

        # ── border lines ──────────────────────────────────────────────────────
        cv2.polylines(annotated, [ws_poly], True, (0, 255, 60), 2)
        cv2.line(annotated, tuple(el), tuple(er), (0, 160, 255), 2)  # entry
        cv2.line(annotated, tuple(xl), tuple(xr), (60, 60, 200), 2)  # exit

        # ── workspace center crosshair (X=0, Y=0) ────────────────────────────
        ctr = self._project_base_to_pixel(
            getattr(config, 'WORKSPACE_OVERLAY_X_OFFSET_MM', 0.0),
            getattr(config, 'WORKSPACE_OVERLAY_Y_OFFSET_MM', 0.0),
            z,
        )
        if ctr is not None:
            cx, cy = int(ctr[0]), int(ctr[1])
            cv2.line(annotated, (cx - 4, cy), (cx + 4, cy), (255, 255, 255), 1)
            cv2.line(annotated, (cx, cy - 4), (cx, cy + 4), (255, 255, 255), 1)


    def draw_base_axis_overlay(self, annotated, x1: int, y1: int, x2: int, y2: int):
        roi_w = max(1, x2 - x1)
        roi_h = max(1, y2 - y1)
        origin = (
            int(x1 + 0.10 * roi_w),
            int(y2 - 0.10 * roi_h),
        )
        axis_len = int(max(26, min(0.08 * roi_w, 0.10 * roi_h)))

        inverse_rotation = np.linalg.inv(self.camera_rotation)
        base_x_cam = inverse_rotation @ np.array((1.0, 0.0, 0.0), dtype=np.float64)
        base_y_cam = inverse_rotation @ np.array((0.0, 1.0, 0.0), dtype=np.float64)

        x_vec = np.array((base_x_cam[0], base_x_cam[1]), dtype=np.float64)
        y_vec = np.array((base_y_cam[0], base_y_cam[1]), dtype=np.float64)

        if np.linalg.norm(x_vec) < 1e-6:
            x_vec = np.array((1.0, 0.0), dtype=np.float64)
        else:
            x_vec = x_vec / np.linalg.norm(x_vec)

        if np.linalg.norm(y_vec) < 1e-6:
            y_vec = np.array((0.0, -1.0), dtype=np.float64)
        else:
            y_vec = y_vec / np.linalg.norm(y_vec)

        x_end = (
            int(round(origin[0] + axis_len * x_vec[0])),
            int(round(origin[1] + axis_len * x_vec[1])),
        )
        y_end = (
            int(round(origin[0] + axis_len * y_vec[0])),
            int(round(origin[1] + axis_len * y_vec[1])),
        )

        cv2.arrowedLine(annotated, origin, x_end, (0, 220, 255), 2, tipLength=0.20)
        cv2.arrowedLine(annotated, origin, y_end, (120, 255, 120), 2, tipLength=0.20)
        cv2.circle(annotated, origin, 3, (255, 255, 255), -1)

        cv2.putText(
            annotated,
            "+Xb",
            (x_end[0] + 4 if x_end[0] >= origin[0] else x_end[0] - 30, x_end[1] - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 220, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            "+Yb",
            (y_end[0] + 4, y_end[1] - 4 if y_end[1] <= origin[1] else y_end[1] + 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (120, 255, 120),
            2,
            cv2.LINE_AA,
        )

    def draw_camera_axis_legend(self, annotated):
        mapping = self.get_axis_mapping_summary()
        legend_lines = (
            "AXIS MAP",
            f"right -> {mapping['cam_x_to_base']}",
            f"down  -> {mapping['cam_y_to_base']}",
            f"away  -> {mapping['cam_z_to_base']}",
        )
        start_x = 12
        start_y = 14
        line_h = 22
        box_w = 260
        box_h = 18 + line_h * len(legend_lines)
        cv2.rectangle(
            annotated,
            (start_x - 6, start_y - 10),
            (start_x - 6 + box_w, start_y - 10 + box_h),
            (20, 20, 20),
            thickness=cv2.FILLED,
        )
        cv2.rectangle(
            annotated,
            (start_x - 6, start_y - 10),
            (start_x - 6 + box_w, start_y - 10 + box_h),
            (180, 180, 180),
            thickness=1,
        )
        for index, line in enumerate(legend_lines):
            y = start_y + 18 + line_h * index
            color = (255, 255, 255) if index == 0 else (220, 255, 220)
            cv2.putText(
                annotated,
                line,
                (start_x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                color,
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                line,
                (start_x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                (40, 40, 40),
                1,
                cv2.LINE_AA,
            )

    def format_axis_expression(self, vector, axis_labels):
        terms = []
        for coeff, label in zip(vector, axis_labels):
            coeff = float(coeff)
            if abs(coeff) < 1e-6:
                continue
            sign = "+" if coeff > 0 else "-"
            mag = abs(coeff)
            if abs(mag - 1.0) < 1e-6:
                terms.append(f"{sign}{label}")
            else:
                terms.append(f"{sign}{mag:.2f}{label}")

        if not terms:
            return "0"

        expression = " ".join(terms)
        return expression[1:] if expression.startswith("+") else expression

    def get_axis_mapping_summary(self):
        row_x = self.format_axis_expression(self.camera_rotation[0], ("x_cam", "y_cam", "z_cam"))
        row_y = self.format_axis_expression(self.camera_rotation[1], ("x_cam", "y_cam", "z_cam"))
        row_z = self.format_axis_expression(self.camera_rotation[2], ("x_cam", "y_cam", "z_cam"))

        cam_x_to_base = self.format_axis_expression(
            self.camera_rotation @ np.array((1.0, 0.0, 0.0), dtype=np.float64),
            ("x_base", "y_base", "z_base"),
        )
        cam_y_to_base = self.format_axis_expression(
            self.camera_rotation @ np.array((0.0, 1.0, 0.0), dtype=np.float64),
            ("x_base", "y_base", "z_base"),
        )
        cam_z_to_base = self.format_axis_expression(
            self.camera_rotation @ np.array((0.0, 0.0, 1.0), dtype=np.float64),
            ("x_base", "y_base", "z_base"),
        )

        return {
            "x_base_eq": f"x_base = {row_x}",
            "y_base_eq": f"y_base = {row_y}",
            "z_base_eq": f"z_base = {row_z}",
            "cam_x_to_base": cam_x_to_base,
            "cam_y_to_base": cam_y_to_base,
            "cam_z_to_base": cam_z_to_base,
        }

    def get_workspace_roi(self, frame_w: int, frame_h: int):
        if not config.WORKSPACE_ROI_ENABLE:
            return 0, 0, frame_w, frame_h

        mx = max(0, int(config.WORKSPACE_MARGIN_X_PX))
        my = max(0, int(config.WORKSPACE_MARGIN_Y_PX))
        x1 = min(frame_w - 1, mx)
        y1 = min(frame_h - 1, my)
        x2 = max(x1 + 1, frame_w - mx)
        y2 = max(y1 + 1, frame_h - my)
        return x1, y1, x2, y2

    def get_robot_exclusion_polygons(self, frame_w: int, frame_h: int):
        polygons = []
        if not config.ROBOT_EXCLUDE_ENABLE:
            return polygons

        for polygon_norm in config.ROBOT_EXCLUDE_POLYGONS_NORM:
            points = []
            for x_norm, y_norm in polygon_norm:
                px = int(round(float(np.clip(x_norm, 0.0, 1.0)) * max(frame_w - 1, 1)))
                py = int(round(float(np.clip(y_norm, 0.0, 1.0)) * max(frame_h - 1, 1)))
                points.append((px, py))
            if len(points) >= 3:
                polygons.append(np.array(points, dtype=np.int32))
        return polygons

    def get_robot_exclusion_mask(self, frame_w: int, frame_h: int):
        mask = np.zeros((frame_h, frame_w), dtype=np.uint8)
        for polygon in self.get_robot_exclusion_polygons(frame_w, frame_h):
            cv2.fillPoly(mask, [polygon], 255)
        return mask

    def detection_overlaps_robot_exclusion(self, frame_w: int, frame_h: int, x1: int, y1: int, x2: int, y2: int, contour=None, exclusion_mask=None) -> bool:
        if not config.ROBOT_EXCLUDE_ENABLE:
            return False

        x1 = max(0, min(frame_w, int(x1)))
        y1 = max(0, min(frame_h, int(y1)))
        x2 = max(0, min(frame_w, int(x2)))
        y2 = max(0, min(frame_h, int(y2)))
        if x2 <= x1 or y2 <= y1:
            return False

        mask = exclusion_mask if exclusion_mask is not None else self.get_robot_exclusion_mask(frame_w, frame_h)
        bbox_mask = mask[y1:y2, x1:x2]
        if bbox_mask.size == 0:
            return False

        bbox_overlap = float(np.count_nonzero(bbox_mask)) / float(bbox_mask.size)
        if bbox_overlap >= config.ROBOT_EXCLUDE_MAX_BBOX_OVERLAP:
            return True

        cx = min(frame_w - 1, max(0, int(round(0.5 * (x1 + x2)))))
        cy = min(frame_h - 1, max(0, int(round(0.5 * (y1 + y2)))))
        if mask[cy, cx] > 0:
            return True

        if contour is not None:
            contour_mask = np.zeros((frame_h, frame_w), dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour.astype(np.int32)], -1, 255, thickness=cv2.FILLED)
            contour_area = float(np.count_nonzero(contour_mask))
            if contour_area > 1.0:
                overlap = float(np.count_nonzero(cv2.bitwise_and(contour_mask, mask))) / contour_area
                if overlap >= config.ROBOT_EXCLUDE_MAX_CONTOUR_OVERLAP:
                    return True

        return False

    def prioritize_workspace_detections(self, frame, detections):
        if not detections:
            return []

        frame_h, frame_w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = self.get_workspace_roi(frame_w, frame_h)
        roi_cx = 0.5 * (rx1 + rx2)
        roi_cy = 0.5 * (ry1 + ry2)
        roi_half_w = max(1.0, 0.5 * (rx2 - rx1))
        roi_half_h = max(1.0, 0.5 * (ry2 - ry1))
        exclusion_mask = self.get_robot_exclusion_mask(frame_w, frame_h)
        prioritized = []

        for det in detections:
            x1 = det["x1"]
            y1 = det["y1"]
            x2 = det["x2"]
            y2 = det["y2"]
            bbox_cx = 0.5 * (x1 + x2)
            bbox_cy = 0.5 * (y1 + y2)
            if not (rx1 <= bbox_cx <= rx2 and ry1 <= bbox_cy <= ry2):
                continue
            if self.detection_overlaps_robot_exclusion(frame_w, frame_h, x1, y1, x2, y2, contour=det.get("primary_contour"), exclusion_mask=exclusion_mask):
                continue

            width = max(1.0, float(x2 - x1))
            height = max(1.0, float(y2 - y1))
            aspect_ratio = max(width, height) / min(width, height)
            center_dx = abs(bbox_cx - roi_cx) / roi_half_w
            center_dy = abs(bbox_cy - roi_cy) / roi_half_h
            center_weight = max(0.2, 1.0 - 0.35 * (center_dx + center_dy))
            size_weight = min(1.5, (width * height) / max(1.0, 0.02 * frame_w * frame_h))

            scored = dict(det)
            scored["score"] = float(det.get("score", width * height)) * center_weight * size_weight / math.sqrt(aspect_ratio)
            prioritized.append(scored)

        prioritized.sort(key=lambda det: det.get("score", 0.0), reverse=True)
        return prioritized

    def update_detection_confirmation(self, track_id: str) -> bool:
        self._confirm_counts[track_id] = self._confirm_counts.get(track_id, 0) + 1
        return self._confirm_counts[track_id] >= config.DETECTION_CONFIRM_FRAMES

    def handle_no_detection(self):
        self.no_detection_count += 1
        if self.no_detection_count >= config.DETECTION_LOST_RESET_FRAMES:
            self.reset_tracking_state()

    def reset_tracking_state(self):
        self._xyz_buffers.clear()
        self._timed_xyz_bufs.clear()
        self.depth_buffer.clear()
        self.last_depth_track_id = None
        self.last_stable_xyz = None
        self._confirm_counts = {}
        self.no_detection_count = 0

    def depth_image_to_meters(self, depth_image):
        if depth_image is None:
            return None

        if depth_image.dtype == np.uint16:
            depth_m = depth_image.astype(np.float32) * 0.001
        elif depth_image.dtype in (np.float32, np.float64):
            depth_m = depth_image.astype(np.float32)
        else:
            return None

        depth_m[~np.isfinite(depth_m)] = 0.0
        return depth_m

    def make_odd(self, value: int) -> int:
        size = max(1, int(value))
        if size % 2 == 0:
            size += 1
        return size

    def estimate_floor_depth_m(self, roi_depth_m, valid_mask):
        valid_values = roi_depth_m[valid_mask]
        if valid_values.size == 0:
            return float(config.FOREGROUND_FLOOR_DEPTH_M)

        dynamic_floor = float(np.percentile(valid_values, config.FOREGROUND_FLOOR_PERCENTILE))
        configured_floor = float(config.FOREGROUND_FLOOR_DEPTH_M)
        if configured_floor <= 0.0:
            return dynamic_floor

        if abs(dynamic_floor - configured_floor) <= 0.20:
            return 0.7 * dynamic_floor + 0.3 * configured_floor
        return dynamic_floor

    def find_depth_foreground_detections(self, frame, depth_image):
        depth_m = self.depth_image_to_meters(depth_image)
        if depth_m is None:
            return []

        frame_h, frame_w = frame.shape[:2]
        if depth_m.shape[:2] != (frame_h, frame_w):
            return []

        rx1, ry1, rx2, ry2 = self.get_workspace_roi(frame_w, frame_h)
        roi_depth = depth_m[ry1:ry2, rx1:rx2]
        if roi_depth.size == 0:
            return []

        valid = (
            np.isfinite(roi_depth)
            & (roi_depth >= config.DEPTH_MIN_M)
            & (roi_depth <= config.DEPTH_MAX_M)
        )
        if np.count_nonzero(valid) < max(200, config.DEPTH_MIN_VALID_PIXELS * 10):
            return []

        floor_depth_m = self.estimate_floor_depth_m(roi_depth, valid)
        filled_depth = np.where(valid, roi_depth, floor_depth_m)
        filled_depth_mm = np.clip(filled_depth * 1000.0, 0, 65535).astype(np.uint16)
        bg_kernel = np.ones(
            (self.make_odd(config.FOREGROUND_BG_CLOSE_PX), self.make_odd(config.FOREGROUND_BG_CLOSE_PX)),
            dtype=np.uint8,
        )
        background_mm = cv2.morphologyEx(filled_depth_mm, cv2.MORPH_CLOSE, bg_kernel, iterations=1)
        background_m = background_mm.astype(np.float32) * 0.001
        height_m = np.where(valid, background_m - roi_depth, 0.0)

        fg_mask = (
            valid
            & (height_m >= config.FOREGROUND_MIN_HEIGHT_M)
            & (height_m <= config.FOREGROUND_MAX_HEIGHT_M)
        ).astype(np.uint8) * 255
        if fg_mask.size == 0:
            return []

        open_kernel = np.ones(
            (self.make_odd(config.FOREGROUND_MASK_OPEN_PX), self.make_odd(config.FOREGROUND_MASK_OPEN_PX)),
            dtype=np.uint8,
        )
        close_kernel = np.ones(
            (self.make_odd(config.FOREGROUND_MASK_CLOSE_PX), self.make_odd(config.FOREGROUND_MASK_CLOSE_PX)),
            dtype=np.uint8,
        )
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, close_kernel, iterations=1)

        exclusion_mask = self.get_robot_exclusion_mask(frame_w, frame_h)[ry1:ry2, rx1:rx2]
        if exclusion_mask.size != 0:
            fg_mask[exclusion_mask > 0] = 0

        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_area = float((rx2 - rx1) * (ry2 - ry1))
        detections = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < config.FOREGROUND_MIN_AREA_PX or area > roi_area * config.FOREGROUND_MAX_AREA_RATIO:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < config.FOREGROUND_MIN_WIDTH_PX or h < config.FOREGROUND_MIN_HEIGHT_PX:
                continue

            border_margin = max(0, int(config.FOREGROUND_BORDER_REJECT_PX))
            if (
                x <= border_margin
                or y <= border_margin
                or (x + w) >= (fg_mask.shape[1] - border_margin)
                or (y + h) >= (fg_mask.shape[0] - border_margin)
            ):
                continue

            full_contour = contour + np.array([[[rx1, ry1]]], dtype=np.int32)
            (_, _), (rect_w, rect_h), _ = cv2.minAreaRect(full_contour)
            short_side = min(rect_w, rect_h)
            long_side = max(rect_w, rect_h)
            if short_side <= 1e-6:
                continue
            aspect_ratio = float(long_side / short_side)
            if aspect_ratio > config.FOREGROUND_MAX_ASPECT_RATIO:
                continue
            if config.FOREGROUND_REQUIRE_BOX_LIKE and not self.is_box_like(full_contour):
                continue

            local_mask = np.zeros(fg_mask.shape, dtype=np.uint8)
            cv2.drawContours(local_mask, [contour], -1, 255, thickness=cv2.FILLED)
            object_heights = height_m[local_mask > 0]
            if object_heights.size == 0:
                continue

            mean_height = float(np.median(object_heights))
            x1 = int(x + rx1)
            y1 = int(y + ry1)
            x2 = int(x + w + rx1)
            y2 = int(y + h + ry1)
            frame_cx = frame_w * 0.5
            frame_cy = frame_h * 0.5
            bbox_cx = x1 + w * 0.5
            bbox_cy = y1 + h * 0.5
            norm_dx = abs(bbox_cx - frame_cx) / max(frame_cx, 1.0)
            norm_dy = abs(bbox_cy - frame_cy) / max(frame_cy, 1.0)
            center_weight = max(0.2, 1.0 - 0.5 * (norm_dx + norm_dy))
            detections.append(
                {
                    "name": config.BBOX_LABEL,
                    "conf": float(np.clip(mean_height / max(config.FOREGROUND_MAX_HEIGHT_M, 1e-3), 0.0, 1.0)),
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "bbox_center": (int(x1 + w / 2), int(y1 + h / 2)),
                    "primary_contour": full_contour,
                    "outer_contour": full_contour,
                    "score": area * max(mean_height, 1e-3) * center_weight / math.sqrt(aspect_ratio),
                }
            )

        detections.sort(key=lambda det: det["score"], reverse=True)
        return detections

    def find_white_rectangle_detections(self, frame):
        frame_h, frame_w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = self.get_workspace_roi(frame_w, frame_h)
        roi = frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return []

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        normalized = self._clahe.apply(gray)

        brightness_mask = cv2.inRange(
            hsv,
            (0, 0, int(config.WHITE_RECT_MIN_BRIGHTNESS)),
            (180, int(config.WHITE_RECT_MAX_SATURATION), 255),
        )
        _, otsu_mask = cv2.threshold(normalized, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        mask = cv2.bitwise_and(brightness_mask, otsu_mask)

        open_kernel = np.ones(
            (self.make_odd(config.WHITE_RECT_MASK_OPEN_PX), self.make_odd(config.WHITE_RECT_MASK_OPEN_PX)),
            dtype=np.uint8,
        )
        close_kernel = np.ones(
            (self.make_odd(config.WHITE_RECT_MASK_CLOSE_PX), self.make_odd(config.WHITE_RECT_MASK_CLOSE_PX)),
            dtype=np.uint8,
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=1)

        exclusion_mask = self.get_robot_exclusion_mask(frame_w, frame_h)[ry1:ry2, rx1:rx2]
        if exclusion_mask.size != 0:
            mask[exclusion_mask > 0] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_area = float(max(1, (rx2 - rx1) * (ry2 - ry1)))
        frame_cx = frame_w * 0.5
        frame_cy = frame_h * 0.5
        detections = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < config.WHITE_RECT_MIN_AREA_PX or area > roi_area * config.WHITE_RECT_MAX_AREA_RATIO:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < config.WHITE_RECT_MIN_WIDTH_PX or h < config.WHITE_RECT_MIN_HEIGHT_PX:
                continue

            border_margin = max(0, int(config.WHITE_RECT_BORDER_REJECT_PX))
            if (
                x <= border_margin
                or y <= border_margin
                or (x + w) >= (mask.shape[1] - border_margin)
                or (y + h) >= (mask.shape[0] - border_margin)
            ):
                continue

            full_contour = contour + np.array([[[rx1, ry1]]], dtype=np.int32)
            corners_uv, rectangularity, aspect_ratio = self.extract_rectangle_corners(
                full_contour,
                frame_gray=frame_gray,
            )
            if corners_uv is None:
                continue
            if rectangularity < config.WHITE_RECT_MIN_RECTANGULARITY:
                continue
            if not (config.WHITE_RECT_MIN_ASPECT_RATIO <= aspect_ratio <= config.WHITE_RECT_MAX_ASPECT_RATIO):
                continue

            contrast = self.compute_contour_contrast(normalized, contour)
            if contrast < config.WHITE_RECT_MIN_CONTRAST:
                continue

            corners_i = np.round(corners_uv).astype(np.int32)
            x1 = int(np.min(corners_i[:, 0]))
            y1 = int(np.min(corners_i[:, 1]))
            x2 = int(np.max(corners_i[:, 0]))
            y2 = int(np.max(corners_i[:, 1]))
            depth_contour = self.find_parent_dark_contour(frame_gray, corners_i)
            if depth_contour is not None:
                dx, dy, dw, dh = cv2.boundingRect(depth_contour)
                depth_bbox = (int(dx), int(dy), int(dx + dw), int(dy + dh))
            else:
                depth_bbox = (x1, y1, x2, y2)
            bbox_cx = float(np.mean(corners_uv[:, 0]))
            bbox_cy = float(np.mean(corners_uv[:, 1]))
            norm_dx = abs(bbox_cx - frame_cx) / max(frame_cx, 1.0)
            norm_dy = abs(bbox_cy - frame_cy) / max(frame_cy, 1.0)
            center_weight = max(0.2, 1.0 - 0.35 * (norm_dx + norm_dy))

            detections.append(
                {
                    "name": "white_rect",
                    "conf": float(np.clip((contrast - config.WHITE_RECT_MIN_CONTRAST) / 32.0, 0.0, 1.0)),
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "bbox_center": (int(round(bbox_cx)), int(round(bbox_cy))),
                    "primary_contour": corners_i.reshape(-1, 1, 2),
                    "outer_contour": corners_i.reshape(-1, 1, 2),
                    "depth_contour": depth_contour,
                    "depth_bbox": depth_bbox,
                    "corners_uv": corners_uv.astype(np.float32),
                    "validated_box_like": True,
                    "score": area * rectangularity * aspect_ratio * center_weight * max(contrast, 1.0),
                }
            )

        detections.sort(key=lambda det: det["score"], reverse=True)
        return detections

    def find_blue_rectangle_detections(self, frame):
        frame_h, frame_w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = self.get_workspace_roi(frame_w, frame_h)
        roi = frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return []

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Blue hue band: H 100-130 (OpenCV 0-180 scale), saturated, not too dark
        blue_mask = cv2.inRange(
            hsv,
            (int(config.BLUE_RECT_HUE_LOW),  int(config.BLUE_RECT_SAT_MIN), int(config.BLUE_RECT_VAL_MIN)),
            (int(config.BLUE_RECT_HUE_HIGH), 255, 255),
        )

        open_k  = self.make_odd(config.BLUE_RECT_MASK_OPEN_PX)
        close_k = self.make_odd(config.BLUE_RECT_MASK_CLOSE_PX)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN,
                                     np.ones((open_k,  open_k),  np.uint8))
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE,
                                     np.ones((close_k, close_k), np.uint8))

        exclusion_mask = self.get_robot_exclusion_mask(frame_w, frame_h)[ry1:ry2, rx1:rx2]
        if exclusion_mask.size != 0:
            blue_mask[exclusion_mask > 0] = 0

        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_area  = float(max(1, (rx2 - rx1) * (ry2 - ry1)))
        frame_cx  = frame_w * 0.5
        frame_cy  = frame_h * 0.5
        sat_chan   = hsv[:, :, 1]
        detections = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < config.BLUE_RECT_MIN_AREA_PX or area > roi_area * config.BLUE_RECT_MAX_AREA_RATIO:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < config.BLUE_RECT_MIN_WIDTH_PX or h < config.BLUE_RECT_MIN_HEIGHT_PX:
                continue

            border_margin = max(0, int(config.BLUE_RECT_BORDER_REJECT_PX))
            if (
                x <= border_margin
                or y <= border_margin
                or (x + w) >= (blue_mask.shape[1] - border_margin)
                or (y + h) >= (blue_mask.shape[0] - border_margin)
            ):
                continue

            # Mean saturation inside the contour as a quality/confidence measure
            obj_mask = np.zeros(roi.shape[:2], np.uint8)
            cv2.drawContours(obj_mask, [contour], -1, 255, -1)
            mean_sat = float(cv2.mean(sat_chan, mask=obj_mask)[0])
            if mean_sat < config.BLUE_RECT_MIN_SAT_MEAN:
                continue

            full_contour = contour + np.array([[[rx1, ry1]]], dtype=np.int32)
            corners_uv, rectangularity, aspect_ratio = self.extract_rectangle_corners(
                full_contour,
                frame_gray=frame_gray,
            )
            if corners_uv is None:
                continue
            if rectangularity < config.BLUE_RECT_MIN_RECTANGULARITY:
                continue
            if not (config.BLUE_RECT_MIN_ASPECT_RATIO <= aspect_ratio <= config.BLUE_RECT_MAX_ASPECT_RATIO):
                continue

            corners_i = np.round(corners_uv).astype(np.int32)
            x1 = int(np.min(corners_i[:, 0]))
            y1 = int(np.min(corners_i[:, 1]))
            x2 = int(np.max(corners_i[:, 0]))
            y2 = int(np.max(corners_i[:, 1]))
            depth_contour = self.find_parent_dark_contour(frame_gray, corners_i)
            if depth_contour is not None:
                dx, dy, dw, dh = cv2.boundingRect(depth_contour)
                depth_bbox = (int(dx), int(dy), int(dx + dw), int(dy + dh))
            else:
                depth_bbox = (x1, y1, x2, y2)
            bbox_cx = float(np.mean(corners_uv[:, 0]))
            bbox_cy = float(np.mean(corners_uv[:, 1]))
            norm_dx = abs(bbox_cx - frame_cx) / max(frame_cx, 1.0)
            norm_dy = abs(bbox_cy - frame_cy) / max(frame_cy, 1.0)
            center_weight = max(0.2, 1.0 - 0.35 * (norm_dx + norm_dy))

            detections.append(
                {
                    "name": "blue_rect",
                    "conf": float(np.clip((mean_sat - config.BLUE_RECT_MIN_SAT_MEAN) / 80.0, 0.0, 1.0)),
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "bbox_center": (int(round(bbox_cx)), int(round(bbox_cy))),
                    "primary_contour": corners_i.reshape(-1, 1, 2),
                    "outer_contour":   corners_i.reshape(-1, 1, 2),
                    "depth_contour":   depth_contour,
                    "depth_bbox":      depth_bbox,
                    "corners_uv":      corners_uv.astype(np.float32),
                    "validated_box_like": True,
                    "score": area * rectangularity * aspect_ratio * center_weight * max(mean_sat, 1.0),
                }
            )

        detections.sort(key=lambda det: det["score"], reverse=True)
        return detections

    def find_orange_blob_detections(self, frame):
        frame_h, frame_w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = self.get_workspace_roi(frame_w, frame_h)
        roi = frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return []

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            (int(config.ORANGE_BLOB_HUE_LOW),  int(config.ORANGE_BLOB_SAT_MIN), int(config.ORANGE_BLOB_VAL_MIN)),
            (int(config.ORANGE_BLOB_HUE_HIGH), 255, 255),
        )

        open_k  = self.make_odd(config.ORANGE_BLOB_MASK_OPEN_PX)
        close_k = self.make_odd(config.ORANGE_BLOB_MASK_CLOSE_PX)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((open_k,  open_k),  np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((close_k, close_k), np.uint8))

        exclusion = self.get_robot_exclusion_mask(frame_w, frame_h)[ry1:ry2, rx1:rx2]
        if exclusion.size != 0:
            mask[exclusion > 0] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_area   = float(max(1, (rx2 - rx1) * (ry2 - ry1)))
        border     = max(0, int(config.ORANGE_BLOB_BORDER_REJECT_PX))
        detections = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < config.ORANGE_BLOB_MIN_AREA_PX or area > roi_area * config.ORANGE_BLOB_MAX_AREA_RATIO:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if (x <= border or y <= border
                    or (x + w) >= (mask.shape[1] - border)
                    or (y + h) >= (mask.shape[0] - border)):
                continue

            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"]) + rx1
            cy = int(M["m01"] / M["m00"]) + ry1

            full_contour = contour + np.array([[[rx1, ry1]]], dtype=np.int32)
            x1 = x + rx1
            y1 = y + ry1
            x2 = x1 + w
            y2 = y1 + h

            detections.append({
                "name": "orange_blob",
                "conf": min(1.0, area / 230.0),   # normalise against expected 30mm area
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "bbox_center": (cx, cy),
                "primary_contour": full_contour,
                "outer_contour":   full_contour,
                "depth_contour":   full_contour,
                "depth_bbox":      (x1, y1, x2, y2),
                "corners_uv":      None,           # centroid only — no corner extraction
                "validated_box_like": True,        # skip box-shape check
                "score": area,
            })

        detections.sort(key=lambda d: d["score"], reverse=True)
        return detections

    def find_orange_square_detections(self, frame):
        frame_h, frame_w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = self.get_workspace_roi(frame_w, frame_h)
        roi = frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return []

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        orange_mask = cv2.inRange(
            hsv,
            (int(config.ORANGE_SQ_HUE_LOW),  int(config.ORANGE_SQ_SAT_MIN), int(config.ORANGE_SQ_VAL_MIN)),
            (int(config.ORANGE_SQ_HUE_HIGH), 255, 255),
        )

        open_k  = self.make_odd(config.ORANGE_SQ_MASK_OPEN_PX)
        close_k = self.make_odd(config.ORANGE_SQ_MASK_CLOSE_PX)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN,
                                       np.ones((open_k,  open_k),  np.uint8))
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE,
                                       np.ones((close_k, close_k), np.uint8))

        exclusion_mask = self.get_robot_exclusion_mask(frame_w, frame_h)[ry1:ry2, rx1:rx2]
        if exclusion_mask.size != 0:
            orange_mask[exclusion_mask > 0] = 0

        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_area  = float(max(1, (rx2 - rx1) * (ry2 - ry1)))
        frame_cx  = frame_w * 0.5
        frame_cy  = frame_h * 0.5
        sat_chan   = hsv[:, :, 1]
        detections = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < config.ORANGE_SQ_MIN_AREA_PX or area > roi_area * config.ORANGE_SQ_MAX_AREA_RATIO:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < config.ORANGE_SQ_MIN_WIDTH_PX or h < config.ORANGE_SQ_MIN_HEIGHT_PX:
                continue

            border_margin = max(0, int(config.ORANGE_SQ_BORDER_REJECT_PX))
            if (
                x <= border_margin
                or y <= border_margin
                or (x + w) >= (orange_mask.shape[1] - border_margin)
                or (y + h) >= (orange_mask.shape[0] - border_margin)
            ):
                continue

            obj_mask = np.zeros(roi.shape[:2], np.uint8)
            cv2.drawContours(obj_mask, [contour], -1, 255, -1)
            mean_sat = float(cv2.mean(sat_chan, mask=obj_mask)[0])
            if mean_sat < config.ORANGE_SQ_MIN_SAT_MEAN:
                continue

            full_contour = contour + np.array([[[rx1, ry1]]], dtype=np.int32)
            corners_uv, rectangularity, aspect_ratio = self.extract_rectangle_corners(
                full_contour,
                frame_gray=frame_gray,
            )
            if corners_uv is None:
                continue
            if rectangularity < config.ORANGE_SQ_MIN_RECTANGULARITY:
                continue
            if not (config.ORANGE_SQ_MIN_ASPECT_RATIO <= aspect_ratio <= config.ORANGE_SQ_MAX_ASPECT_RATIO):
                continue

            corners_i = np.round(corners_uv).astype(np.int32)
            x1 = int(np.min(corners_i[:, 0]))
            y1 = int(np.min(corners_i[:, 1]))
            x2 = int(np.max(corners_i[:, 0]))
            y2 = int(np.max(corners_i[:, 1]))
            depth_contour = self.find_parent_dark_contour(frame_gray, corners_i)
            if depth_contour is not None:
                dx, dy, dw, dh = cv2.boundingRect(depth_contour)
                depth_bbox = (int(dx), int(dy), int(dx + dw), int(dy + dh))
            else:
                depth_bbox = (x1, y1, x2, y2)
            bbox_cx = float(np.mean(corners_uv[:, 0]))
            bbox_cy = float(np.mean(corners_uv[:, 1]))
            norm_dx = abs(bbox_cx - frame_cx) / max(frame_cx, 1.0)
            norm_dy = abs(bbox_cy - frame_cy) / max(frame_cy, 1.0)
            center_weight = max(0.2, 1.0 - 0.35 * (norm_dx + norm_dy))

            detections.append(
                {
                    "name": "orange_sq",
                    "conf": float(np.clip((mean_sat - config.ORANGE_SQ_MIN_SAT_MEAN) / 80.0, 0.0, 1.0)),
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "bbox_center": (int(round(bbox_cx)), int(round(bbox_cy))),
                    "primary_contour": corners_i.reshape(-1, 1, 2),
                    "outer_contour":   corners_i.reshape(-1, 1, 2),
                    "depth_contour":   depth_contour,
                    "depth_bbox":      depth_bbox,
                    "corners_uv":      corners_uv.astype(np.float32),
                    "validated_box_like": True,
                    "score": area * rectangularity * center_weight * max(mean_sat, 1.0),
                }
            )

        detections.sort(key=lambda det: det["score"], reverse=True)
        return detections

    def find_parent_dark_contour(self, frame_gray, label_corners):
        x, y, w, h = cv2.boundingRect(label_corners.reshape(-1, 1, 2).astype(np.int32))
        if w <= 0 or h <= 0:
            return None

        frame_h, frame_w = frame_gray.shape[:2]
        pad_x = max(18, int(1.8 * w))
        pad_y = max(18, int(1.8 * h))
        rx1 = max(0, x - pad_x)
        ry1 = max(0, y - pad_y)
        rx2 = min(frame_w, x + w + pad_x)
        ry2 = min(frame_h, y + h + pad_y)
        if rx2 <= rx1 or ry2 <= ry1:
            return None

        roi = frame_gray[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return None

        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        _, dark_mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        dark_mask = cv2.morphologyEx(
            dark_mask,
            cv2.MORPH_CLOSE,
            np.ones((7, 7), dtype=np.uint8),
            iterations=2,
        )
        dark_mask = cv2.morphologyEx(
            dark_mask,
            cv2.MORPH_OPEN,
            self._kernel_5,
            iterations=1,
        )

        contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        label_center = np.mean(label_corners.astype(np.float32), axis=0)
        label_center_local = (float(label_center[0] - rx1), float(label_center[1] - ry1))
        label_area = float(max(1, w * h))
        best_contour = None
        best_score = 0.0

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 2.0 * label_area:
                continue

            bx, by, bw, bh = cv2.boundingRect(contour)
            if bw <= w or bh <= h:
                continue

            if cv2.pointPolygonTest(contour, label_center_local, False) < 0:
                continue

            rect = cv2.minAreaRect(contour)
            rect_w, rect_h = float(abs(rect[1][0])), float(abs(rect[1][1]))
            if rect_w <= 1e-6 or rect_h <= 1e-6:
                continue

            rectangularity = area / max(rect_w * rect_h, 1e-6)
            if rectangularity < 0.55:
                continue

            score = area * rectangularity
            if score > best_score:
                best_score = score
                best_contour = contour

        if best_contour is None:
            return None

        return best_contour + np.array([[[rx1, ry1]]], dtype=np.int32)

    def extract_rectangle_corners(self, contour, frame_gray):
        area = float(cv2.contourArea(contour))
        if area < config.WHITE_RECT_MIN_AREA_PX:
            return None, 0.0, 0.0

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            return None, 0.0, 0.0

        approx = cv2.approxPolyDP(contour, config.WHITE_RECT_POLY_EPSILON_SCALE * perimeter, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            corners = approx.reshape(4, 2).astype(np.float32)
        else:
            box = cv2.boxPoints(cv2.minAreaRect(contour))
            corners = box.astype(np.float32)

        ordered = self.order_quad_corners(corners)
        refined = self.refine_quad_corners_subpix(frame_gray, ordered)

        contour_area = float(cv2.contourArea(refined.reshape(-1, 1, 2)))
        rect_w = np.linalg.norm(refined[1] - refined[0])
        rect_h = np.linalg.norm(refined[3] - refined[0])
        short_side = min(rect_w, rect_h)
        long_side = max(rect_w, rect_h)
        if short_side <= 1e-6:
            return None, 0.0, 0.0

        rectangularity = contour_area / max(rect_w * rect_h, 1e-6)
        aspect_ratio = long_side / short_side
        return refined, rectangularity, aspect_ratio

    def compute_contour_contrast(self, gray_roi, contour):
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return 0.0

        local_contour = contour - np.array([[[x, y]]], dtype=np.int32)
        roi = gray_roi[y:y + h, x:x + w]
        if roi.size == 0:
            return 0.0

        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(mask, [local_contour], -1, 255, thickness=cv2.FILLED)
        if np.count_nonzero(mask) == 0:
            return 0.0

        pad = max(4, int(0.08 * max(w, h)))
        rx1 = max(0, x - pad)
        ry1 = max(0, y - pad)
        rx2 = min(gray_roi.shape[1], x + w + pad)
        ry2 = min(gray_roi.shape[0], y + h + pad)
        ring_roi = gray_roi[ry1:ry2, rx1:rx2]
        if ring_roi.size == 0:
            return 0.0

        ring_mask = np.zeros(ring_roi.shape[:2], dtype=np.uint8)
        shifted = contour - np.array([[[rx1, ry1]]], dtype=np.int32)
        cv2.rectangle(ring_mask, (0, 0), (ring_roi.shape[1] - 1, ring_roi.shape[0] - 1), 255, thickness=cv2.FILLED)
        cv2.drawContours(ring_mask, [shifted], -1, 0, thickness=cv2.FILLED)

        inner_mean = float(np.mean(roi[mask > 0]))
        ring_values = ring_roi[ring_mask > 0]
        if ring_values.size == 0:
            return inner_mean
        return inner_mean - float(np.mean(ring_values))

    def order_quad_corners(self, corners):
        corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        center = np.mean(corners, axis=0)
        angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
        order = np.argsort(angles)
        sorted_corners = corners[order]
        start = np.argmin(np.sum(sorted_corners, axis=1))
        sorted_corners = np.roll(sorted_corners, -int(start), axis=0)
        return sorted_corners

    def refine_quad_corners_subpix(self, gray, corners):
        corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
        window = self.make_odd(config.WHITE_RECT_SUBPIX_WINDOW)
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.01,
        )
        try:
            refined = cv2.cornerSubPix(gray, corners, (window, window), (-1, -1), criteria)
        except cv2.error:
            refined = corners
        return refined.reshape(-1, 2)

    def find_yolo_detections(self, frame):
        if self.model is None:
            return []

        frame_h, frame_w = frame.shape[:2]
        exclusion_mask = self.get_robot_exclusion_mask(frame_w, frame_h)

        results = self.model.predict(
            source=frame,
            conf=self.CONF_THRES,
            iou=self.YOLO_IOU,
            imgsz=self.YOLO_IMGSZ,
            max_det=self.YOLO_MAX_DET,
            verbose=False,
            device=self.YOLO_DEVICE,
        )
        if len(results) == 0:
            return []

        result = results[0]
        boxes = result.boxes
        if boxes is None:
            return []

        detections = []
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int).tolist()
            conf = float(box.conf[0].item())
            cls_id = int(box.cls[0].item())
            name = self.model.names.get(cls_id, str(cls_id))
            if self.TARGET_CLASS is not None and name != self.TARGET_CLASS:
                continue
            if self.detection_overlaps_robot_exclusion(frame_w, frame_h, x1, y1, x2, y2, exclusion_mask=exclusion_mask):
                continue
            detections.append(
                {
                    "name": name,
                    "conf": conf,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "outer_contour": None,
                    "score": conf,
                }
            )
        return detections

    def find_bbox_detections(self, frame):
        contours = self.find_frame_box_contours(frame)
        frame_h, frame_w = frame.shape[:2]
        exclusion_mask = self.get_robot_exclusion_mask(frame_w, frame_h)
        detections = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if self.detection_overlaps_robot_exclusion(frame_w, frame_h, x, y, x + w, y + h, contour=contour, exclusion_mask=exclusion_mask):
                continue
            detections.append(
                {
                    "name": config.BBOX_LABEL,
                    "conf": 1.0,
                    "x1": int(x),
                    "y1": int(y),
                    "x2": int(x + w),
                    "y2": int(y + h),
                    "bbox_center": (int(x + w / 2), int(y + h / 2)),
                    "primary_contour": contour,
                    "outer_contour": contour,
                    "score": float(cv2.contourArea(contour)),
                }
            )
        detections.sort(key=lambda det: det["score"], reverse=True)
        return self.deduplicate_detections(detections)

    def find_frame_box_contours(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        normalized = self._clahe.apply(gray)
        blur = cv2.GaussianBlur(normalized, (5, 5), 0)
        _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        edges = cv2.Canny(blur, config.CANNY_LOW, config.CANNY_HIGH)
        edges = cv2.morphologyEx(
            edges,
            cv2.MORPH_CLOSE,
            self._kernel_5,
            iterations=2,
        )

        contour_sets = []
        for binary in (th, 255 - th, edges):
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour_sets.extend(contours)

        frame_area = float(frame.shape[0] * frame.shape[1])
        max_area = frame_area * config.BBOX_FRAME_MAX_AREA_RATIO
        valid = []
        for contour in contour_sets:
            area = float(cv2.contourArea(contour))
            if area < config.BBOX_FRAME_MIN_AREA or area > max_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < config.BBOX_FRAME_MIN_WIDTH_PX or h < config.BBOX_FRAME_MIN_HEIGHT_PX:
                continue
            if not self.bbox_inside_frame(frame.shape[1], frame.shape[0], x, y, x + w, y + h):
                continue
            if not self.is_box_like(contour):
                continue
            valid.append(contour)
        return valid

    def deduplicate_detections(self, detections):
        kept = []
        for det in detections:
            if any(self.bbox_iou(det, existing) > config.BBOX_NMS_IOU for existing in kept):
                continue
            kept.append(det)
        return kept

    def bbox_iou(self, det_a, det_b):
        ax1, ay1, ax2, ay2 = det_a["x1"], det_a["y1"], det_a["x2"], det_a["y2"]
        bx1, by1, bx2, by2 = det_b["x1"], det_b["y1"], det_b["x2"], det_b["y2"]

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = float(inter_w * inter_h)

        area_a = float(max(0, ax2 - ax1) * max(0, ay2 - ay1))
        area_b = float(max(0, bx2 - bx1) * max(0, by2 - by1))
        union = area_a + area_b - inter_area
        if union <= 1e-6:
            return 0.0
        return inter_area / union

    def make_track_id(self, name: str, x1: int, y1: int, x2: int, y2: int) -> str:
        grid = max(1, config.CONVEYOR_TRACK_GRID_PX if config.CONVEYOR_MODE
                   else config.TRACK_GRID_PX)
        bbox_cu = int((x1 + x2) / 2)
        bbox_cv = int((y1 + y2) / 2)
        return f"{name}_{bbox_cu // grid}_{bbox_cv // grid}"

    def bbox_inside_frame(self, frame_w: int, frame_h: int, x1: int, y1: int, x2: int, y2: int) -> bool:
        margin = max(0, int(config.FRAME_MARGIN_PX))
        return (
            x1 >= margin
            and y1 >= margin
            and x2 <= frame_w - margin
            and y2 <= frame_h - margin
        )

    def draw_rejected_detection(self, annotated, x1: int, y1: int, x2: int, y2: int, label: str):
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 165, 255), 2)
        cv2.putText(
            annotated,
            label,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 165, 255),
            2,
            cv2.LINE_AA,
        )

    def extract_primary_contour(self, frame, x1, y1, x2, y2):
        bbox_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
        contour = self.extract_contour_from_roi(
            frame,
            x1,
            y1,
            x2,
            y2,
            shrink_ratio=config.BOX_ROI_SHRINK,
            pad_px=0,
        )
        return bbox_center, contour

    def extract_outer_contour(self, frame, x1, y1, x2, y2):
        return self.extract_contour_from_roi(
            frame,
            x1,
            y1,
            x2,
            y2,
            shrink_ratio=0.0,
            pad_px=6,
        )

    def extract_contour_from_roi(self, frame, x1, y1, x2, y2, shrink_ratio: float, pad_px: int):
        frame_h, frame_w = frame.shape[:2]
        w = x2 - x1
        h = y2 - y1

        rx1 = max(0, int(x1 - pad_px + shrink_ratio * w))
        ry1 = max(0, int(y1 - pad_px + shrink_ratio * h))
        rx2 = min(frame_w, int(x2 + pad_px - shrink_ratio * w))
        ry2 = min(frame_h, int(y2 + pad_px - shrink_ratio * h))

        if rx2 <= rx1 or ry2 <= ry1:
            return None

        roi = frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        contours_a, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_b, _ = cv2.findContours(255 - th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_area = 0.0
        for contours in (contours_a, contours_b):
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > best_area:
                    best_area = area
                    best_contour = contour

        if best_contour is None or best_area < config.BOX_MIN_CONTOUR_AREA:
            return None

        return best_contour + np.array([[[rx1, ry1]]], dtype=np.int32)

    def is_box_like(self, contour) -> bool:
        if contour is None:
            return False

        area = cv2.contourArea(contour)
        if area < config.BOX_MIN_CONTOUR_AREA:
            return False

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            return False

        approx = cv2.approxPolyDP(contour, config.BOX_POLY_EPSILON_SCALE * perimeter, True)
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            return False

        (_, _), (width, height), _ = cv2.minAreaRect(contour)
        short_side = min(width, height)
        long_side = max(width, height)
        if short_side <= 1e-6:
            return False

        rectangular_area = width * height
        rectangularity = area / rectangular_area if rectangular_area > 1e-6 else 0.0
        aspect_ratio = long_side / short_side

        return (
            rectangularity >= config.BOX_MIN_RECTANGULARITY
            and aspect_ratio <= config.BOX_MAX_ASPECT_RATIO
        )

    def contour_center_or_bbox(self, bbox_center, contour):
        if contour is None:
            return bbox_center

        (cu, cv), _, _ = cv2.minAreaRect(contour)
        if np.isfinite(cu) and np.isfinite(cv):
            return int(round(cu)), int(round(cv))

        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-6:
            return bbox_center

        cu = int(moments["m10"] / moments["m00"])
        cv = int(moments["m01"] / moments["m00"])
        return cu, cv

    def refine_center_from_roi(self, frame, x1, y1, x2, y2):
        bbox_center, contour = self.extract_primary_contour(frame, x1, y1, x2, y2)
        return self.contour_center_or_bbox(bbox_center, contour)

    def get_depth_image(self):
        if config.FAKE_DEPTH_ENABLE:
            return None

        with self._depth_lock:
            msg = self.latest_depth_msg
            stamp = self.latest_depth_stamp

        if msg is None:
            return None

        if stamp is not None:
            age_sec = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if age_sec > 0.5:
                self.get_logger().warn(
                    "Depth image stale ({:.2f}s) — check RealSense connection".format(age_sec),
                    throttle_duration_sec=2.0,
                )
                return None

        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as ex:
            self.get_logger().warning(f"Depth conversion failed: {ex}")
            return None

    def get_depth_meters(
        self,
        depth_image,
        u: int,
        v: int,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        track_id: str,
        contour=None,
    ) -> Optional[float]:
        if depth_image is None:
            return None

        values_m = self.collect_depth_values_meters(depth_image, u, v, x1, y1, x2, y2, contour)
        z_m = self.robust_depth_from_values(values_m)
        if z_m is None:
            return None

        return self.smooth_depth_estimate(track_id, z_m)

    def collect_depth_values_meters(
        self,
        depth_image,
        u: int,
        v: int,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        contour=None,
    ):
        h, w = depth_image.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return np.empty(0, dtype=np.float32)

        contour_values = np.empty(0, dtype=np.float32)
        if config.DEPTH_USE_CONTOUR_MASK and contour is not None:
            contour_values = self.extract_valid_depth_values_from_contour(depth_image, contour)
            if contour_values.size >= config.DEPTH_MIN_VALID_PIXELS:
                return contour_values

        roi_half_w = max(config.CENTER_WINDOW, int((x2 - x1) * config.DEPTH_BOX_SCALE * 0.5))
        roi_half_h = max(config.CENTER_WINDOW, int((y2 - y1) * config.DEPTH_BOX_SCALE * 0.5))

        rx1 = max(0, max(x1, u - roi_half_w))
        rx2 = min(w, min(x2, u + roi_half_w + 1))
        ry1 = max(0, max(y1, v - roi_half_h))
        ry2 = min(h, min(y2, v + roi_half_h + 1))

        inner_values = self.extract_valid_depth_values(depth_image, rx1, ry1, rx2, ry2)

        cx1 = max(0, u - config.CENTER_WINDOW)
        cx2 = min(w, u + config.CENTER_WINDOW + 1)
        cy1 = max(0, v - config.CENTER_WINDOW)
        cy2 = min(h, v + config.CENTER_WINDOW + 1)
        center_values = self.extract_valid_depth_values(depth_image, cx1, cy1, cx2, cy2)

        if contour_values.size > 0:
            if inner_values.size == 0:
                inner_values = contour_values
            else:
                inner_values = np.concatenate((contour_values, inner_values))

        if inner_values.size >= config.DEPTH_MIN_VALID_PIXELS:
            return inner_values
        if center_values.size >= max(3, config.DEPTH_MIN_VALID_PIXELS // 2):
            return center_values
        if inner_values.size == 0:
            return center_values
        if center_values.size == 0:
            return inner_values
        return np.concatenate((inner_values, center_values))

    def extract_valid_depth_values(self, depth_image, x1: int, y1: int, x2: int, y2: int):
        if x2 <= x1 or y2 <= y1:
            return np.empty(0, dtype=np.float32)

        roi = depth_image[y1:y2, x1:x2]
        if roi.size == 0:
            return np.empty(0, dtype=np.float32)

        return self.filter_depth_values(roi)

    def extract_valid_depth_values_from_contour(self, depth_image, contour):
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return np.empty(0, dtype=np.float32)

        roi = depth_image[y:y + h, x:x + w]
        if roi.size == 0:
            return np.empty(0, dtype=np.float32)

        local_contour = contour - np.array([[[x, y]]], dtype=np.int32)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(mask, [local_contour], -1, 255, thickness=cv2.FILLED)

        erode_px = max(0, int(config.DEPTH_CONTOUR_ERODE_PX))
        if erode_px > 0:
            kernel = np.ones((erode_px, erode_px), dtype=np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)

        values = roi[mask > 0]
        if values.size == 0:
            return np.empty(0, dtype=np.float32)

        return self.filter_depth_values(values)

    def filter_depth_values(self, values):
        if values.size == 0:
            return np.empty(0, dtype=np.float32)

        if values.dtype == np.uint16:
            depth_values = values[values > 0].astype(np.float32) * 0.001
        elif values.dtype in (np.float32, np.float64):
            depth_values = values[np.isfinite(values) & (values > 0)].astype(np.float32)
        else:
            return np.empty(0, dtype=np.float32)

        if depth_values.size == 0:
            return depth_values

        return depth_values[(depth_values >= config.DEPTH_MIN_M) & (depth_values <= config.DEPTH_MAX_M)]

    def robust_depth_from_values(self, values_m) -> Optional[float]:
        if values_m.size < max(3, config.DEPTH_MIN_VALID_PIXELS // 2):
            return None

        low = np.percentile(values_m, config.DEPTH_TRIM_LOW_PERCENT)
        high = np.percentile(values_m, config.DEPTH_TRIM_HIGH_PERCENT)
        trimmed = values_m[(values_m >= low) & (values_m <= high)]
        if trimmed.size < max(3, values_m.size // 3):
            trimmed = values_m

        median = float(np.median(trimmed))
        mad = float(np.median(np.abs(trimmed - median)))
        if mad > 1e-6:
            limit = config.DEPTH_MAD_SCALE * 1.4826 * mad
            filtered = trimmed[np.abs(trimmed - median) <= limit]
            if filtered.size >= max(3, config.DEPTH_MIN_VALID_PIXELS // 2):
                trimmed = filtered

        z_m = float(np.median(trimmed))
        if z_m < config.DEPTH_MIN_M or z_m > config.DEPTH_MAX_M:
            return None
        return z_m

    def smooth_depth_estimate(self, track_id: str, z_m: float) -> Optional[float]:
        if self.last_depth_track_id != track_id:
            self.depth_buffer.clear()
            self.last_depth_track_id = track_id

        if len(self.depth_buffer) >= 2:
            history = np.array(self.depth_buffer, dtype=np.float32)
            history_median = float(np.median(history))
            if abs(z_m - history_median) > config.DEPTH_MAX_JUMP_M:
                return history_median

        self.depth_buffer.append(z_m)
        history = np.array(self.depth_buffer, dtype=np.float32)
        return float(np.median(history))

    def pixel_to_camera_xyz_mm(self, u: int, v: int, z_m: float):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None

        z_mm = z_m * 1000.0
        x_mm = ((u - self.cx) * z_mm) / self.fx
        y_mm = ((v - self.cy) * z_mm) / self.fy
        return x_mm, y_mm, z_mm

    def estimate_object_size(self, contour, x1: int, y1: int, x2: int, y2: int, z_mm: float):
        bbox_w_px = max(0.0, float(x2 - x1))
        bbox_h_px = max(0.0, float(y2 - y1))

        obj_w_px = bbox_w_px
        obj_h_px = bbox_h_px
        size_source = "bbox"

        if contour is not None:
            (_, _), (rect_w_px, rect_h_px), _ = cv2.minAreaRect(contour)
            rect_w_px = float(abs(rect_w_px))
            rect_h_px = float(abs(rect_h_px))
            if rect_w_px > 1e-6 and rect_h_px > 1e-6:
                obj_w_px = rect_w_px
                obj_h_px = rect_h_px
                size_source = "outer_contour"

        bbox_long_px, bbox_short_px = sorted((bbox_w_px, bbox_h_px), reverse=True)
        obj_long_px, obj_short_px = sorted((obj_w_px, obj_h_px), reverse=True)

        pixel_scale_mm = z_mm * 0.5 * ((1.0 / self.fx) + (1.0 / self.fy))
        obj_long_mm = obj_long_px * pixel_scale_mm
        obj_short_mm = obj_short_px * pixel_scale_mm

        return {
            "bbox_size_px": (bbox_long_px, bbox_short_px),
            "obj_size_px": (obj_long_px, obj_short_px),
            "obj_size_mm": (obj_long_mm, obj_short_mm),
            "size_source": size_source,
        }

    def detect_ee_marker(self, frame):
        """Detect white laser dot on EE tip. Returns (u, v) centroid or None.

        Must be called with the RAW frame.
        White in HSV: any hue, S < sat_max, V > val_min.
        Among size-passing blobs the brightest is selected.
        Blobs within ±8px of the workspace crosshair are excluded.
        """
        # ── DEBUG: widened thresholds + console logging for first 100 calls ──
        self._ee_debug_count = getattr(self, '_ee_debug_count', 0) + 1
        debug_active = self._ee_debug_count <= 100
        do_print     = debug_active and (self._ee_debug_count % 30 == 1)

        if debug_active:
            sat_max  = 80    # widened (config: EE_LASER_SAT_MAX=50)
            val_min  = 180   # widened (config: EE_LASER_VAL_MIN=210)
            max_area = 500   # widened (config: EE_LASER_MAX_AREA=300)
        else:
            sat_max  = int(config.EE_LASER_SAT_MAX)
            val_min  = int(config.EE_LASER_VAL_MIN)
            max_area = int(config.EE_LASER_MAX_AREA)
        # ─────────────────────────────────────────────────────────────────────

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, val_min), (180, sat_max, 255))

        open_k  = 3
        close_k = 5
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((open_k,  open_k),  np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((close_k, close_k), np.uint8))

        # Exclude ±8px around the workspace-centre crosshair (also white)
        crosshair_px = None
        if self.fx is not None:
            ox = getattr(config, 'WORKSPACE_OVERLAY_X_OFFSET_MM', 0.0)
            oy = getattr(config, 'WORKSPACE_OVERLAY_Y_OFFSET_MM', 0.0)
            crosshair_px = self._project_base_to_pixel(ox, oy, config.WORKSPACE_PICK_Z_MM)
        if crosshair_px is not None:
            ccx, ccy = int(crosshair_px[0]), int(crosshair_px[1])
            mh, mw = mask.shape[:2]
            mask[max(0, ccy - 8):min(mh, ccy + 9),
                 max(0, ccx - 8):min(mw, ccx + 9)] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        v_chan = hsv[:, :, 2]
        best = None
        best_brightness = 0.0
        tmp_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        debug_areas = []
        for c in contours:
            area = float(cv2.contourArea(c))
            passed = config.EE_LASER_MIN_AREA <= area <= max_area
            if do_print:
                debug_areas.append(f"{area:.0f}({'ok' if passed else 'skip'})")
            if not passed:
                continue
            tmp_mask[:] = 0
            cv2.drawContours(tmp_mask, [c], -1, 255, cv2.FILLED)
            mean_v = float(cv2.mean(v_chan, mask=tmp_mask)[0])
            if mean_v > best_brightness:
                best_brightness = mean_v
                best = c

        # ── raw centroid from brightest passing blob ───────────────────────────
        raw = None
        if best is not None:
            M = cv2.moments(best)
            if M["m00"] != 0:
                raw = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # ── workspace bounds filter (±20px around ws_poly bounding box) ───────
        if raw is not None:
            ws_poly = getattr(self, '_ws_poly', None)
            if ws_poly is not None:
                pts = ws_poly.reshape(-1, 2)
                x_min = int(pts[:, 0].min()) - 20
                x_max = int(pts[:, 0].max()) + 20
                y_min = int(pts[:, 1].min()) - 20
                y_max = int(pts[:, 1].max()) + 20
                if not (x_min < raw[0] < x_max and y_min < raw[1] < y_max):
                    raw = None

        # ── temporal smoothing: jump filter + median over last N frames ────────
        if not hasattr(self, '_ee_history'):
            self._ee_history = deque(maxlen=config.EE_LASER_SMOOTH_FRAMES)

        result = None
        if raw is not None:
            # Jump filter: reject if too far from last accepted position
            if self._ee_history:
                lx, ly = self._ee_history[-1]
                jump = ((raw[0] - lx) ** 2 + (raw[1] - ly) ** 2) ** 0.5
                if jump > config.EE_LASER_MAX_JUMP_PX:
                    raw = None   # noise spike — discard

        if raw is not None:
            self._ee_history.append(raw)

        if self._ee_history:
            xs = sorted(p[0] for p in self._ee_history)
            ys = sorted(p[1] for p in self._ee_history)
            n  = len(xs)
            result = (xs[n // 2], ys[n // 2])

        if do_print:
            xhair = (f"({int(crosshair_px[0])},{int(crosshair_px[1])})"
                     if crosshair_px is not None else "None")
            print(f"[EE debug #{self._ee_debug_count}]  "
                  f"{len(contours)} contours  areas={debug_areas}  "
                  f"crosshair_excl={xhair}  raw={raw}  result={result}")

        return result

    def _build_T_cam_to_base(self):
        """Build the 4×4 homogeneous transform T_cam_to_base.

        p_base = T_cam_to_base @ [p_cam; 1]

        Returns (T_cam_to_base, T_base_to_cam) both as (4,4) float64 arrays.
        The inverse is computed analytically: T_inv = [[R.T, -R.T @ t], [0,0,0,1]]
        which avoids numerical error from np.linalg.inv on a rotation matrix.
        """
        R = self.build_camera_rotation_matrix()
        t = np.array([config.CAM_TX_MM, config.CAM_TY_MM, config.CAM_TZ_MM],
                     dtype=np.float64)

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3,  3] = t

        T_inv = np.eye(4, dtype=np.float64)
        T_inv[:3, :3] = R.T
        T_inv[:3,  3] = -R.T @ t

        self.get_logger().info(
            f"T_cam_to_base built  t=({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}) mm  "
            f"det(R)={np.linalg.det(R):.6f}"
        )
        return T, T_inv

    def build_camera_rotation_matrix(self):
        legacy = self.legacy_rotation_matrix(config.CAMERA_TRANSFORM_MODE)

        if config.CAMERA_USE_DIRECT_MATRIX:
            try:
                matrix = np.array(config.CAMERA_DIRECT_MATRIX, dtype=np.float64)
                if matrix.shape != (3, 3):
                    raise ValueError(f"expected 3x3 matrix, got {matrix.shape}")
            except Exception as ex:
                self.get_logger().warning(
                    f"Invalid CAMERA_DIRECT_MATRIX, falling back to legacy mode: {ex}"
                )
                matrix = legacy
        else:
            matrix = legacy

        fine = self.rpy_rotation_matrix(
            config.CAM_FINE_ROLL_DEG,
            config.CAM_FINE_PITCH_DEG,
            config.CAM_FINE_YAW_DEG,
        )
        matrix = fine @ matrix

        self.get_logger().info(
            "Camera transform rotation ready: "
            f"mode={config.CAMERA_TRANSFORM_MODE}, "
            f"direct_matrix={config.CAMERA_USE_DIRECT_MATRIX}, "
            f"fine_rpy_deg=({config.CAM_FINE_ROLL_DEG:.2f}, "
            f"{config.CAM_FINE_PITCH_DEG:.2f}, {config.CAM_FINE_YAW_DEG:.2f})"
        )
        return matrix

    def build_plane_homography(self):
        if not config.PLANE_HOMOGRAPHY_ENABLE:
            return None

        try:
            matrix = np.array(config.PLANE_HOMOGRAPHY_MATRIX, dtype=np.float64)
            if matrix.shape != (3, 3):
                raise ValueError(f"expected 3x3 matrix, got {matrix.shape}")
        except Exception as ex:
            self.get_logger().warning(f"Invalid PLANE_HOMOGRAPHY_MATRIX: {ex}")
            return None

        self.get_logger().info("Plane homography enabled for base x,y estimation")
        return matrix

    def _validate_config(self):
        errors = []
        if config.Z_MIN >= config.Z_MAX:
            errors.append(f"Z_MIN ({config.Z_MIN}) must be < Z_MAX ({config.Z_MAX})")
        if config.X_LIMIT <= 0:
            errors.append(f"X_LIMIT must be > 0, got {config.X_LIMIT}")
        if config.Y_LIMIT <= 0:
            errors.append(f"Y_LIMIT must be > 0, got {config.Y_LIMIT}")
        if config.DEPTH_MIN_M >= config.DEPTH_MAX_M:
            errors.append("DEPTH_MIN_M must be < DEPTH_MAX_M")
        valid_modes = ("yolo", "white_rectangle", "blue_rectangle", "orange_square", "orange_blob", "depth_foreground", "bbox_only")
        if config.DETECTION_MODE.lower() not in valid_modes:
            errors.append(f"DETECTION_MODE '{config.DETECTION_MODE}' not in {valid_modes}")
        if config.CAMERA_USE_DIRECT_MATRIX:
            try:
                m = np.array(config.CAMERA_DIRECT_MATRIX, dtype=np.float64)
                if m.shape != (3, 3):
                    errors.append(f"CAMERA_DIRECT_MATRIX must be 3x3, got {m.shape}")
                else:
                    det = float(np.linalg.det(m))
                    if abs(abs(det) - 1.0) > 0.05:
                        errors.append(
                            f"CAMERA_DIRECT_MATRIX det={det:.4f}, expected ±1.0 "
                            f"— matrix is not a valid rotation. Re-run calibrate_transform.py"
                        )
            except Exception as ex:
                errors.append(f"CAMERA_DIRECT_MATRIX invalid: {ex}")
        if config.PLANE_HOMOGRAPHY_ENABLE:
            try:
                m = np.array(config.PLANE_HOMOGRAPHY_MATRIX, dtype=np.float64)
                if m.shape != (3, 3):
                    errors.append(f"PLANE_HOMOGRAPHY_MATRIX must be 3x3, got {m.shape}")
            except Exception as ex:
                errors.append(f"PLANE_HOMOGRAPHY_MATRIX invalid: {ex}")
        for name, mn, mx in [
            ("THETA1", config.THETA1_MIN, config.THETA1_MAX),
            ("THETA2", config.THETA2_MIN, config.THETA2_MAX),
            ("THETA3", config.THETA3_MIN, config.THETA3_MAX),
        ]:
            if mn >= mx:
                errors.append(f"{name}_MIN ({mn}) must be < {name}_MAX ({mx})")
        if errors:
            msg = "Config validation failed:\n" + "\n".join(f"  - {e}" for e in errors)
            self.get_logger().fatal(msg)
            raise ValueError(msg)
        self.get_logger().info("Config validation passed")

    def legacy_rotation_matrix(self, mode: str):
        mode = mode.upper()
        if mode == "A":
            return np.array(
                ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
                dtype=np.float64,
            )
        if mode == "B":
            return np.array(
                ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
                dtype=np.float64,
            )
        if mode == "C":
            return np.array(
                ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)),
                dtype=np.float64,
            )
        return np.array(
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)),
            dtype=np.float64,
        )

    def rpy_rotation_matrix(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        cx, sx = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw), math.sin(yaw)

        rx = np.array(((1.0, 0.0, 0.0), (0.0, cx, -sx), (0.0, sx, cx)), dtype=np.float64)
        ry = np.array(((cy, 0.0, sy), (0.0, 1.0, 0.0), (-sy, 0.0, cy)), dtype=np.float64)
        rz = np.array(((cz, -sz, 0.0), (sz, cz, 0.0), (0.0, 0.0, 1.0)), dtype=np.float64)
        return rz @ ry @ rx

    def image_points_to_base_xy(self, points_uv):
        if self.plane_homography is None:
            return None

        points = np.asarray(points_uv, dtype=np.float32).reshape(-1, 1, 2)
        projected = cv2.perspectiveTransform(points, self.plane_homography.astype(np.float64))
        return projected.reshape(-1, 2).astype(np.float64)

    def estimate_plane_center_and_yaw(self, corners_uv):
        base_corners = self.image_points_to_base_xy(corners_uv)
        if base_corners is None or len(base_corners) != 4:
            return None, None, None

        center_xy = np.mean(base_corners, axis=0)
        edges = (
            base_corners[1] - base_corners[0],
            base_corners[2] - base_corners[1],
        )
        lengths = [np.linalg.norm(edge) for edge in edges]
        major_edge = edges[int(np.argmax(lengths))]
        yaw_deg = math.degrees(math.atan2(float(major_edge[1]), float(major_edge[0])))
        return (
            (float(center_xy[0]), float(center_xy[1])),
            float(yaw_deg),
            base_corners,
        )

    def estimate_rectangle_pose(self, corners_uv):
        if (
            not config.RECT_POSE_ENABLE
            or self.camera_matrix is None
            or self.dist_coeffs is None
            or config.RECT_REAL_WIDTH_MM <= 0.0
            or config.RECT_REAL_HEIGHT_MM <= 0.0
        ):
            return None

        image_points = np.asarray(corners_uv, dtype=np.float64).reshape(4, 1, 2)
        half_w = 0.5 * float(config.RECT_REAL_WIDTH_MM)
        half_h = 0.5 * float(config.RECT_REAL_HEIGHT_MM)
        object_points = np.array(
            [
                (-half_w, -half_h, 0.0),
                (half_w, -half_h, 0.0),
                (half_w, half_h, 0.0),
                (-half_w, half_h, 0.0),
            ],
            dtype=np.float64,
        )

        try:
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE,
            )
        except cv2.error:
            return None

        if not ok:
            return None

        pose_cam = tuple(float(v) for v in tvec.reshape(3))
        pose_base = self.camera_to_base_mm(*pose_cam)
        rotation_obj_cam, _ = cv2.Rodrigues(rvec)
        rotation_obj_base = self.camera_rotation @ rotation_obj_cam
        yaw_deg = math.degrees(math.atan2(rotation_obj_base[1, 0], rotation_obj_base[0, 0]))

        return {
            "pose_cam_xyz": pose_cam,
            "pose_base_xyz": pose_base,
            "pose_yaw_deg": float(yaw_deg),
        }

    def camera_to_base_mm(self, x_cam: float, y_cam: float, z_cam: float):
        p_cam = np.array([x_cam, y_cam, z_cam, 1.0], dtype=np.float64)
        p_base = self.T_cam_to_base @ p_cam
        return float(p_base[0]), float(p_base[1]), float(p_base[2])

    def _project_base_to_pixel(self, x_b: float, y_b: float, z_b: float):
        """Project a base-frame point (mm) to image pixel (u, v) via T_base_to_cam
        and pinhole projection.  Returns None if camera not calibrated or point
        is behind the camera (z_cam <= 0).
        """
        if self.fx is None:
            return None
        p_cam = self.T_base_to_cam @ np.array([x_b, y_b, z_b, 1.0], dtype=np.float64)
        x_c, y_c, z_c = p_cam[:3]
        if z_c <= 1.0:
            return None
        # ratio x_c/z_c is unitless (mm/mm) so fx/fy pixel values work directly
        u = int(round(self.fx * x_c / z_c + self.cx))
        v = int(round(self.fy * y_c / z_c + self.cy))
        return u, v

    def camera_to_parallel_mm(self, x_cam: float, y_cam: float, z_cam: float):
        camera_xyz = np.array((x_cam, y_cam, z_cam), dtype=np.float64)
        parallel_xyz = self.camera_rotation @ camera_xyz
        return float(parallel_xyz[0]), float(parallel_xyz[1]), float(parallel_xyz[2])

    def update_xyz_average(self, track_id: str, xyz_now):
        if track_id not in self._xyz_buffers:
            self._xyz_buffers[track_id] = deque(maxlen=config.AVG_FRAME_COUNT)
            self._timed_xyz_bufs[track_id] = deque(maxlen=config.AVG_FRAME_COUNT)

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._xyz_buffers[track_id].append(xyz_now)
        self._timed_xyz_bufs[track_id].append((now_sec, xyz_now[0], xyz_now[1], xyz_now[2]))

        if len(self._xyz_buffers[track_id]) < config.AVG_FRAME_COUNT:
            return None

        arr = np.array(self._xyz_buffers[track_id], dtype=np.float64)
        xyz_avg = np.median(arr, axis=0)
        return float(xyz_avg[0]), float(xyz_avg[1]), float(xyz_avg[2])

    def _estimate_vy(self, timed_buf=None) -> float:
        """Estimate conveyor velocity along Y axis (mm/s).

        Uses linear regression over the timed XYZ buffer.  The result is
        sanity-checked against CONVEYOR_VY_MIN/MAX_MM_S:
          - Fewer than 3 samples or |vy| < MIN  → belt is stopped, return 0.
          - |vy| > MAX                           → regression outlier,
                                                   fall back to design speed.
        Sign is negative (belt moves in –Y direction).
        """
        buf_src = timed_buf if timed_buf is not None else deque()
        if len(buf_src) < 3:
            return 0.0

        buf = np.array(buf_src, dtype=np.float64)
        t   = buf[:, 0] - buf[0, 0]
        y   = buf[:, 2]
        A   = np.vstack([t, np.ones(len(t))]).T
        vy, _ = np.linalg.lstsq(A, y, rcond=None)[0]

        if not config.CONVEYOR_MODE:
            return float(vy)

        vy_abs = abs(vy)
        if vy_abs < config.CONVEYOR_VY_MIN_MM_S:
            return 0.0                              # belt stopped / very slow
        if vy_abs > config.CONVEYOR_VY_MAX_MM_S:
            return -config.CONVEYOR_BELT_SPEED_MM_S # outlier → use design speed
        return float(vy)

    def is_stable(self, xyz_avg) -> bool:
        if self.last_stable_xyz is None:
            self.last_stable_xyz = xyz_avg
            return False

        dx = abs(xyz_avg[0] - self.last_stable_xyz[0])
        dy = abs(xyz_avg[1] - self.last_stable_xyz[1])
        dz = abs(xyz_avg[2] - self.last_stable_xyz[2])
        stable = (
            dx <= config.STABLE_THRESH_X_MM
            and dy <= config.STABLE_THRESH_Y_MM
            and dz <= config.STABLE_THRESH_Z_MM
        )
        if not stable:
            self.last_stable_xyz = xyz_avg
        return stable

    def check_workspace(self, x, y, z):
        from delta_common.fk_ik import check_workspace as fk_check_workspace
        return fk_check_workspace(x, y, z)

    def validate_target(self, x_base, y_base, z_base):
        if not self.check_workspace(x_base, y_base, z_base):
            return None, None, None, False, "OUTSIDE_WORKSPACE"

        st_ik, t1, t2, t3 = delta_calcInverse(x_base, y_base, z_base, e, f, re, rf)
        if st_ik != 0:
            return None, None, None, False, "IK_FAILED"

        if not (
            config.THETA1_MIN <= t1 <= config.THETA1_MAX
            and config.THETA2_MIN <= t2 <= config.THETA2_MAX
            and config.THETA3_MIN <= t3 <= config.THETA3_MAX
        ):
            return (t1, t2, t3), None, None, False, "JOINT_LIMIT"

        st_fk, x_fk, y_fk, z_fk = delta_calcForward(t1, t2, t3, e, f, re, rf)
        if st_fk != 0:
            return (t1, t2, t3), None, None, False, "FK_FAILED"

        fk_xyz = (x_fk, y_fk, z_fk)
        fk_err = math.sqrt(
            (x_base - x_fk) ** 2
            + (y_base - y_fk) ** 2
            + (z_base - z_fk) ** 2
        )

        if fk_err > config.FK_VERIFY_TOL_MM:
            return (t1, t2, t3), fk_xyz, fk_err, False, "FK_MISMATCH"

        return (t1, t2, t3), fk_xyz, fk_err, True, "OK"

    def maybe_print_result(self, best):
        now = time.time()
        if now - self.last_print_time < config.PRINT_COOLDOWN_SEC:
            return

        self.last_print_time = now

        if config.PURE_CAMERA_TEST_ENABLE:
            if len(self._frame_times) >= 2:
                dt = self._frame_times[-1] - self._frame_times[0]
                fps = (len(self._frame_times) - 1) / dt if dt > 0 else 0.0
            else:
                fps = 0.0
            avg_ms = (sum(self._proc_times) / len(self._proc_times) * 1000) if self._proc_times else 0.0
            print("\n========== CAMERA OBJECT ==========")
            print(f"fps          : {fps:.1f}  proc: {avg_ms:.1f} ms")
            print(f"name         : {best['name']}")
            print(f"conf         : {best['conf']:.2f}")
            print(f"center_uv    : ({best['u']}, {best['v']})")
            print(
                f"cam_xyz      : ({best['x_cam']:.2f}, {best['y_cam']:.2f}, {best['z_cam']:.2f}) mm"
            )
            print(
                "cam_parallel : "
                f"({best['x_cam_parallel']:.2f}, {best['y_cam_parallel']:.2f}, {best['z_cam_parallel']:.2f}) mm"
            )
            if best["pose_cam_xyz"] is not None:
                print(
                    f"pose_cam_xyz : ({best['pose_cam_xyz'][0]:.2f}, {best['pose_cam_xyz'][1]:.2f}, {best['pose_cam_xyz'][2]:.2f}) mm"
                )
            else:
                print("pose_cam_xyz : None")
            if best["pose_yaw_deg"] is not None:
                print(f"yaw_deg      : {best['pose_yaw_deg']:.2f}")
            else:
                print("yaw_deg      : None")
            print(f"status       : {best['reason']}")
            print("===================================\n")
            return

        if config.SIMPLE_RESULT_PRINT:
            print("\n============= OBJECT =============")
            print(f"name         : {best['name']}")
            print(f"conf         : {best['conf']:.2f}")
            print(f"center_uv    : ({best['u']}, {best['v']})")
            print(
                f"obj_xyz      : ({best['x_base']:.2f}, {best['y_base']:.2f}, {best['z_base']:.2f}) mm"
            )
            print(
                f"cam_xyz      : ({best['x_cam']:.2f}, {best['y_cam']:.2f}, {best['z_cam']:.2f}) mm"
            )
            if best["plane_xy"] is not None:
                print(
                    f"center_xy    : ({best['plane_xy'][0]:.2f}, {best['plane_xy'][1]:.2f}) mm [{best['xy_source']}]"
                )
            else:
                print(f"center_xy    : None [{best['xy_source']}]")
            if best["pose_yaw_deg"] is not None:
                print(f"yaw_deg      : {best['pose_yaw_deg']:.2f}")
            else:
                print("yaw_deg      : None")
            if best["pose_base_xyz"] is not None:
                print(
                    f"pose_base_xyz: ({best['pose_base_xyz'][0]:.2f}, {best['pose_base_xyz'][1]:.2f}, {best['pose_base_xyz'][2]:.2f}) mm"
                )
            else:
                print("pose_base_xyz: None")
            print(f"status       : {best['reason']}")
            print("==================================\n")
            return

        print("\n================ RESULT ================")
        print(f"name         : {best['name']}")
        print(f"conf         : {best['conf']:.2f}")
        print(f"center_uv    : ({best['u']}, {best['v']})")
        print(
            f"raw_cam_xyz  : ({best['x_cam']:.2f}, {best['y_cam']:.2f}, {best['z_cam']:.2f}) mm"
        )
        print(
            "cam_parallel_xyz : "
            f"({best['x_cam_parallel']:.2f}, {best['y_cam_parallel']:.2f}, {best['z_cam_parallel']:.2f}) mm"
        )
        mapping = self.get_axis_mapping_summary()
        print("axis_map:")
        print(f"  {mapping['x_base_eq']}")
        print(f"  {mapping['y_base_eq']}")
        print(f"  {mapping['z_base_eq']}")
        print("motion_hint:")
        print(f"  image right -> {mapping['cam_x_to_base']}")
        print(f"  image down  -> {mapping['cam_y_to_base']}")
        print(f"  move away   -> {mapping['cam_z_to_base']}")
        print(f"xy_source    : {best['xy_source']}")
        if best["plane_xy"] is not None:
            print(
                f"plane_xy     : ({best['plane_xy'][0]:.2f}, {best['plane_xy'][1]:.2f}) mm"
            )
        else:
            print("plane_xy     : None")
        if best["yaw_deg"] is not None:
            print(f"rect_yaw_deg : {best['yaw_deg']:.2f}")
        else:
            print("rect_yaw_deg : None")
        print(
            f"avg_base_xyz : ({best['x_base']:.2f}, {best['y_base']:.2f}, {best['z_base']:.2f}) mm"
        )
        print(
            f"bbox_size_px : ({best['bbox_size_px'][0]:.1f}, {best['bbox_size_px'][1]:.1f})"
        )
        print(
            f"obj_size_px  : ({best['obj_size_px'][0]:.1f}, {best['obj_size_px'][1]:.1f}) [{best['size_source']}]"
        )
        print(
            f"obj_size_mm  : ({best['obj_size_mm'][0]:.2f}, {best['obj_size_mm'][1]:.2f}) mm"
        )
        print(
            f"in_workspace : {self.check_workspace(best['x_base'], best['y_base'], best['z_base'])}"
        )

        if best["ik_deg"] is not None:
            print(
                f"theta_deg    : ({best['ik_deg'][0]:.2f}, {best['ik_deg'][1]:.2f}, {best['ik_deg'][2]:.2f})"
            )
        else:
            print("theta_deg    : None")

        if best["fk_xyz"] is not None:
            print(
                f"fk_xyz       : ({best['fk_xyz'][0]:.2f}, {best['fk_xyz'][1]:.2f}, {best['fk_xyz'][2]:.2f}) mm"
            )
        else:
            print("fk_xyz       : None")

        if best["fk_err"] is not None:
            print(f"fk_error     : {best['fk_err']:.2f} mm")
        else:
            print("fk_error     : None")

        if best["pose_cam_xyz"] is not None:
            print(
                f"pose_cam_xyz : ({best['pose_cam_xyz'][0]:.2f}, {best['pose_cam_xyz'][1]:.2f}, {best['pose_cam_xyz'][2]:.2f}) mm"
            )
            print(
                f"pose_base_xyz: ({best['pose_base_xyz'][0]:.2f}, {best['pose_base_xyz'][1]:.2f}, {best['pose_base_xyz'][2]:.2f}) mm"
            )
            print(f"pose_yaw_deg : {best['pose_yaw_deg']:.2f}")
        else:
            print("pose_cam_xyz : None")
            print("pose_base_xyz: None")
            print("pose_yaw_deg : None")

        print(f"move_allow   : {best['allowed']}")
        print(f"status       : {best['reason']}")
        print("========================================\n")

    def fake_motor_command(self, best):
        if config.VISION_ONLY_ENABLE:
            return

        now = time.time()
        if now - self.last_fake_move_time < config.FAKE_MOVE_COOLDOWN_SEC:
            return

        if not best["allowed"] or best["ik_deg"] is None:
            return

        t1, t2, t3 = best["ik_deg"]

        print("----------- FAKE MOTOR COMMAND -----------")
        print(
            f"FAKE MOVE XYZ   : ({best['x_base']:.2f}, {best['y_base']:.2f}, {best['z_base']:.2f}) mm"
        )
        print(f"FAKE MOVE THETA : ({t1:.2f}, {t2:.2f}, {t3:.2f}) deg")

        if best["fk_xyz"] is not None:
            print(
                f"FAKE FK XYZ     : ({best['fk_xyz'][0]:.2f}, {best['fk_xyz'][1]:.2f}, {best['fk_xyz'][2]:.2f}) mm"
            )

        if best["fk_err"] is not None:
            print(f"FAKE FK ERR     : {best['fk_err']:.2f} mm")

        print("------------------------------------------")
        self.last_fake_move_time = now

    def _calibrate_offset_srv(self, request, response):
        self.run_cam_offset_calibration()
        response.success = True
        response.message = "Calibration started — results printed after 30 detections"
        return response

    def run_cam_offset_calibration(self) -> None:
        """
        Start camera offset calibration.
        Place an object at the physical robot centre (X=0, Y=0) before calling.
        After 30 detected frames the node prints the required CAM_TX_MM / CAM_TY_MM
        corrections. The robot is not moved.
        """
        self._cal_samples = []
        self._cal_active = True
        self.get_logger().info(
            "Cam offset calibration started — place object at robot centre (X=0, Y=0). "
            "Collecting 30 detections..."
        )

    def _collect_cal_sample(self, best: dict) -> None:
        if not self._cal_active:
            return
        self._cal_samples.append((best["x_base"], best["y_base"]))
        n = len(self._cal_samples)
        if n < 30:
            if n % 5 == 0:
                self.get_logger().info(f"Calibration: {n}/30 samples...")
            return
        self._cal_active = False
        mean_x = sum(s[0] for s in self._cal_samples) / n
        mean_y = sum(s[1] for s in self._cal_samples) / n
        new_tx = config.CAM_TX_MM - mean_x
        new_ty = config.CAM_TY_MM - mean_y
        self.get_logger().info(
            f"\n====== CAM OFFSET CALIBRATION RESULT ======\n"
            f"  Samples     : {n}\n"
            f"  mean x_base : {mean_x:+.2f} mm\n"
            f"  mean y_base : {mean_y:+.2f} mm\n"
            f"  Corrections needed:\n"
            f"    CAM_TX_MM  {config.CAM_TX_MM:.2f} + ({-mean_x:+.2f}) = {new_tx:.2f}\n"
            f"    CAM_TY_MM  {config.CAM_TY_MM:.2f} + ({-mean_y:+.2f}) = {new_ty:.2f}\n"
            f"  → Update config.py:\n"
            f"    CAM_TX_MM = {new_tx:.2f}\n"
            f"    CAM_TY_MM = {new_ty:.2f}\n"
            f"==========================================="
        )

    def destroy_node(self):
        if self.VIEW_IMAGE:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeltaCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
