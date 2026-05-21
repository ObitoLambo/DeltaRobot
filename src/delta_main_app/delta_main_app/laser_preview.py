#!/usr/bin/env python3
"""
laser_preview.py — Live laser dot detection preview

Run this BEFORE the experiment to verify:
  - The laser dot is visible to the camera
  - The HSV colour thresholds detect it correctly
  - The dot centroid pixel is stable when the robot is stationary

Opens an OpenCV window showing:
  - Raw camera feed
  - HSV mask overlay (white = detected area)
  - Green circle around the detected centroid
  - Pixel coordinates and area printed on the frame

Press  Q  to quit.

Usage
-----
    ros2 run delta_main_app laser_preview
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, Tuple

from delta_common import config

# ── Paste the same thresholds used in laser_accuracy_experiment.py ────────────
LASER_COLOR = "red"   # "red" or "green"

_RED_HSV = [
    ((0,   160, 120), (10,  255, 255)),
    ((168, 160, 120), (180, 255, 255)),
]
_GREEN_HSV = [
    ((40, 160, 120), (80, 255, 255)),
]

DOT_MIN_AREA_PX = 4
DOT_MAX_AREA_PX = 1800


def detect_laser_dot(bgr: np.ndarray, color: str) -> Tuple[Optional[Tuple[float, float]], float, np.ndarray]:
    """
    Returns:
        centroid (u, v) or None
        area in pixels
        mask image (for overlay)
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    ranges = _RED_HSV if color == "red" else _GREEN_HSV

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

    k3 = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k3)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if DOT_MIN_AREA_PX <= cv2.contourArea(c) <= DOT_MAX_AREA_PX]

    if not valid:
        return None, 0.0, mask

    best = max(valid, key=cv2.contourArea)
    area = cv2.contourArea(best)
    M = cv2.moments(best)
    if M["m00"] == 0:
        return None, 0.0, mask

    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return (cx, cy), area, mask


class LaserPreview(Node):

    def __init__(self):
        super().__init__("laser_preview")
        self.bridge = CvBridge()
        self._latest: Optional[np.ndarray] = None

        self.create_subscription(Image, config.COLOR_TOPIC, self._cb, 1)
        self.get_logger().info(
            f"Subscribing to {config.COLOR_TOPIC}\n"
            f"Laser colour: {LASER_COLOR}   Press Q in the preview window to quit."
        )

    def _cb(self, msg: Image) -> None:
        self._latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def spin(self) -> None:
        cv2.namedWindow("Laser Preview", cv2.WINDOW_NORMAL)
        cv2.namedWindow("HSV Mask",      cv2.WINDOW_NORMAL)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.03)

            if self._latest is None:
                continue

            frame = self._latest.copy()
            centroid, area, mask = detect_laser_dot(frame, LASER_COLOR)

            # ── Annotate the colour frame ──────────────────────────────────
            if centroid is not None:
                u, v = int(centroid[0]), int(centroid[1])
                cv2.circle(frame, (u, v), 12, (0, 255, 0), 2)       # outer ring
                cv2.circle(frame, (u, v),  2, (0, 255, 0), -1)      # centre dot
                cv2.putText(
                    frame,
                    f"DETECTED  px=({u},{v})  area={area:.0f}px",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0), 2,
                )
            else:
                cv2.putText(
                    frame,
                    "NOT DETECTED — check laser and HSV thresholds",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2,
                )

            cv2.putText(
                frame,
                f"Color: {LASER_COLOR}  |  Q = quit",
                (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1,
            )

            # Show mask in blue tint for clarity
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_bgr[:, :, 0] = 0   # zero out R and G channels
            mask_bgr[:, :, 1] = 0
            cv2.putText(
                mask_bgr,
                "HSV mask (blue = detected region)",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 1,
            )

            cv2.imshow("Laser Preview", frame)
            cv2.imshow("HSV Mask",      mask_bgr)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = LaserPreview()
    try:
        node.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
