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
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
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
        self.last_confirm_track_id = None
        self.confirm_track_count = 0
        self.no_detection_count = 0

        self.camera_rotation = self.build_camera_rotation_matrix()
        self.camera_translation = np.array(
            [config.CAM_TX_MM, config.CAM_TY_MM, config.CAM_TZ_MM],
            dtype=np.float64,
        )
        self.plane_homography = self.build_plane_homography()

        self.xyz_buffer = deque(maxlen=config.AVG_FRAME_COUNT)
        self.depth_buffer = deque(maxlen=config.DEPTH_HISTORY_SIZE)
        self.last_track_id = None
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
        self._pub_target = self.create_publisher(PointStamped, "/delta/target_xyz", 10)
        self._pub_status = self.create_publisher(String, "/delta/detection_status", 10)

        mode = "FAKE DEPTH" if config.FAKE_DEPTH_ENABLE else "REAL DEPTH"
        self.get_logger().info(
            f"Delta camera system ready ({mode}, detection={self.DETECTION_MODE})"
        )

        if self.VIEW_IMAGE:
            cv2.namedWindow("Delta Camera", cv2.WINDOW_NORMAL)

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

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as ex:
            self.get_logger().error(f"Color conversion failed: {ex}")
            return

        annotated, best = self.process_frame(frame)

        if best is not None:
            self.maybe_print_result(best)
            self.fake_motor_command(best)

        if self.VIEW_IMAGE:
            cv2.imshow("Delta Camera", annotated)
            cv2.waitKey(1)

    def process_frame(self, frame):
        annotated = frame.copy()
        best = None
        pure_camera_mode = bool(config.PURE_CAMERA_TEST_ENABLE)
        depth_image = self.get_depth_image()
        frame_h, frame_w = frame.shape[:2]
        self.draw_workspace_overlay(annotated, frame_w, frame_h)
        if config.DRAW_CAMERA_AXIS_LEGEND:
            self.draw_camera_axis_legend(annotated)

        detections = self.generate_detections(frame, depth_image)
        if not detections:
            self.handle_no_detection()
            return annotated, best

        self.no_detection_count = 0

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
            if z_m is None:
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
            if config.DETECT_ONLY_IN_WORKSPACE and not self.check_workspace(*workspace_xyz):
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
                    if not self.is_stable(xyz_avg):
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

            label_color = (0, 255, 0) if allowed else (0, 0, 255)
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
            else:
                label = (
                    f"{name} {conf:.2f} "
                    f"C=({u},{v}) "
                    f"BASE=({x_use:.1f},{y_use:.1f},{z_use:.1f}) "
                    f"{theta_text}{reason}"
                )
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
            }

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
                pt = PointStamped()
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.header.frame_id = "robot_base"
                pt.point.x = best["x_base"] / 1000.0
                pt.point.y = best["y_base"] / 1000.0
                pt.point.z = best["z_base"] / 1000.0
                self._pub_target.publish(pt)

        return annotated, best

    def generate_detections(self, frame, depth_image=None):
        if self.DETECTION_MODE == "white_rectangle":
            rect_detections = self.find_white_rectangle_detections(frame)
            return self.prioritize_workspace_detections(frame, rect_detections)
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
        if self.last_confirm_track_id != track_id:
            self.last_confirm_track_id = track_id
            self.confirm_track_count = 1
        else:
            self.confirm_track_count += 1
        return self.confirm_track_count >= config.DETECTION_CONFIRM_FRAMES

    def handle_no_detection(self):
        self.no_detection_count += 1
        if self.no_detection_count >= config.DETECTION_LOST_RESET_FRAMES:
            self.reset_tracking_state()

    def reset_tracking_state(self):
        self.xyz_buffer.clear()
        self.depth_buffer.clear()
        self.last_track_id = None
        self.last_depth_track_id = None
        self.last_stable_xyz = None
        self.last_confirm_track_id = None
        self.confirm_track_count = 0
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
        grid = max(1, config.TRACK_GRID_PX)
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
        if config.FAKE_DEPTH_ENABLE:
            return config.FAKE_DEPTH_M

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
        valid_modes = ("yolo", "white_rectangle", "depth_foreground", "bbox_only")
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
        camera_xyz = np.array((x_cam, y_cam, z_cam), dtype=np.float64)
        base_xyz = self.camera_rotation @ camera_xyz + self.camera_translation
        return float(base_xyz[0]), float(base_xyz[1]), float(base_xyz[2])

    def camera_to_parallel_mm(self, x_cam: float, y_cam: float, z_cam: float):
        camera_xyz = np.array((x_cam, y_cam, z_cam), dtype=np.float64)
        parallel_xyz = self.camera_rotation @ camera_xyz
        return float(parallel_xyz[0]), float(parallel_xyz[1]), float(parallel_xyz[2])

    def update_xyz_average(self, track_id: str, xyz_now):
        if self.last_track_id != track_id:
            self.xyz_buffer.clear()
            self.last_stable_xyz = None
            self.last_track_id = track_id

        self.xyz_buffer.append(xyz_now)
        if len(self.xyz_buffer) < config.AVG_FRAME_COUNT:
            return None

        arr = np.array(self.xyz_buffer, dtype=np.float64)
        xyz_avg = np.median(arr, axis=0)
        return float(xyz_avg[0]), float(xyz_avg[1]), float(xyz_avg[2])

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
        return (
            abs(x) <= config.X_LIMIT
            and abs(y) <= config.Y_LIMIT
            and config.Z_MIN <= z <= config.Z_MAX
        )

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
            print("\n========== CAMERA OBJECT ==========")
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
