#!/usr/bin/env python3

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO


class RealSenseYoloDetector(Node):
    def __init__(self):
        super().__init__('realsense_yolo_detector')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('annotated_topic', '/yolo/annotated_image')
        self.declare_parameter('detection_text_topic', '/yolo/detections_text')
        self.declare_parameter('view_image', True)
        self.declare_parameter('device', 'cpu')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = float(
            self.get_parameter('conf_threshold').get_parameter_value().double_value
        )
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value
        detection_text_topic = self.get_parameter('detection_text_topic').get_parameter_value().string_value
        self.view_image = self.get_parameter('view_image').get_parameter_value().bool_value
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'YOLO model loaded successfully')

        self.bridge = CvBridge()

        # Latest depth and intrinsics
        self.latest_depth_msg: Optional[Image] = None
        self.depth_scale = 0.001  # default if depth image is uint16 in mm
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Publishers
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.text_pub = self.create_publisher(String, detection_text_topic, 10)

        # Subscribers
        self.color_sub = self.create_subscription(Image, color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.get_logger().info(f'Subscribed color topic: {color_topic}')
        self.get_logger().info(f'Subscribed depth topic: {depth_topic}')
        self.get_logger().info(f'Subscribed camera info topic: {camera_info_topic}')

        if self.view_image:
            cv2.namedWindow('YOLO Detection', cv2.WINDOW_NORMAL)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        # K = [fx, 0, cx,
        #      0, fy, cy,
        #      0, 0, 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Image) -> None:
        self.latest_depth_msg = msg

    def get_depth_meters(self, u: int, v: int) -> Optional[float]:
        if self.latest_depth_msg is None:
            return None

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                self.latest_depth_msg, desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().warning(f'Failed to convert depth image: {e}')
            return None

        h, w = depth_image.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None

        # small window around center pixel
        win = 2
        x1 = max(0, u - win)
        x2 = min(w, u + win + 1)
        y1 = max(0, v - win)
        y2 = min(h, v + win + 1)

        roi = depth_image[y1:y2, x1:x2]

        if depth_image.dtype == np.uint16:
            valid = roi[roi > 0]
            if valid.size == 0:
                return None
            return float(np.median(valid)) * 0.001

        if depth_image.dtype in (np.float32, np.float64):
            valid = roi[np.isfinite(roi) & (roi > 0.0)]
            if valid.size == 0:
                return None
            return float(np.median(valid))

        return None
    def pixel_to_3d(self, u: int, v: int, z: float):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return (x, y, z)

    def color_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert color image: {e}')
            return

        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            verbose=False,
            device=self.device
        )

        annotated = frame.copy()
        lines = []

        if len(results) > 0:
            result = results[0]
            boxes = result.boxes

            if boxes is not None:
                for box in boxes:
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy.tolist()

                    cls_id = int(box.cls[0].item()) if box.cls is not None else -1
                    conf = float(box.conf[0].item()) if box.conf is not None else 0.0
                    class_name = self.model.names.get(cls_id, str(cls_id))

                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)

                    depth_m = self.get_depth_meters(u, v)
                    xyz = self.pixel_to_3d(u, v, depth_m) if depth_m is not None else None

                    # Draw bbox
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(annotated, (u, v), 4, (0, 0, 255), -1)

                    label = f'{class_name} {conf:.2f}'

                    if depth_m is not None:
                        label += f' | Z={depth_m:.2f}m'

                    if xyz is not None:
                        x3d, y3d, z3d = xyz
                        line = (
                            f'class={class_name}, conf={conf:.2f}, '
                            f'pixel=({u},{v}), xyz=({x3d:.3f},{y3d:.3f},{z3d:.3f})'
                        )
                    else:
                        line = (
                            f'class={class_name}, conf={conf:.2f}, '
                            f'pixel=({u},{v}), xyz=(None,None,None)'
                        )

                    lines.append(line)

                    # Draw label background
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
                    y_text = max(y1 - 10, 20)
                    cv2.rectangle(
                        annotated,
                        (x1, y_text - th - 8),
                        (x1 + tw + 8, y_text + 4),
                        (0, 255, 0),
                        -1
                    )
                    cv2.putText(
                        annotated,
                        label,
                        (x1 + 4, y_text),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 0, 0),
                        2,
                        cv2.LINE_AA
                    )

        # Publish text summary
        msg_text = String()
        msg_text.data = '\n'.join(lines) if lines else 'No detections'
        self.text_pub.publish(msg_text)

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')

        # Show local window
        if self.view_image:
            cv2.imshow('YOLO Detection', annotated)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.view_image:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()