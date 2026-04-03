#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

COLOR_TOPIC = "/camera/camera/color/image_raw"
YOLO_MODEL = "yolov8n.pt"
CONF_THRES = 0.5
TARGET_CLASS = None
VIEW_IMAGE = True

_running = True


class DetectCenterOnly(Node):
    def __init__(self):
        super().__init__("detect_center_only")

        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL)

        self.sub_color = self.create_subscription(
            Image, COLOR_TOPIC, self.color_callback, 10
        )

        self.get_logger().info("Detect center only node ready")

        if VIEW_IMAGE:
            cv2.namedWindow("Detect Center Only", cv2.WINDOW_NORMAL)

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

                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)

                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(annotated, (u, v), 5, (0, 0, 255), -1)

                    label = f"{name} {conf:.2f} C=({u},{v})"
                    cv2.putText(
                        annotated,
                        label,
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA
                    )

                    print(f"{name}: box=({x1},{y1},{x2},{y2}) center=({u},{v}) conf={conf:.2f}")

        if VIEW_IMAGE:
            cv2.imshow("Detect Center Only", annotated)
            cv2.waitKey(1)

    def destroy_node(self):
        if VIEW_IMAGE:
            cv2.destroyAllWindows()
        super().destroy_node()


def _sig_handler(sig, frame):
    global _running
    _running = False


def main(args=None):
    global _running
    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    rclpy.init(args=args)
    node = DetectCenterOnly()

    try:
        while rclpy.ok() and _running:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()