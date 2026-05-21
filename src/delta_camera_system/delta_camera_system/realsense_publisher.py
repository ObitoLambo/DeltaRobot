import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time
import numpy as np
import pyrealsense2 as rs


class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')

        self.pub_color = self.create_publisher(Image, '/camera/camera/color/image_raw', 1)
        self.pub_depth = self.create_publisher(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', 1
        )
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', 1)

        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

        profile = self._pipeline.start(cfg)
        self._align = rs.align(rs.stream.color)

        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self._camera_info = self._build_camera_info(intr)

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.get_logger().info('RealSense publisher started (pyrealsense2 direct)')

    def _capture_loop(self):
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=5000)
            except RuntimeError:
                self.get_logger().warn('wait_for_frames timed out, retrying...')
                continue

            aligned = self._align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            stamp = self._rs_stamp_to_ros(color_frame.get_timestamp())

            self.pub_color.publish(self._to_image_msg(
                np.asanyarray(color_frame.get_data()), 'bgr8', stamp
            ))
            self.pub_depth.publish(self._to_image_msg(
                np.asanyarray(depth_frame.get_data()), '16UC1', stamp
            ))
            self._camera_info.header.stamp = stamp
            self.pub_info.publish(self._camera_info)

    def _rs_stamp_to_ros(self, ms: float) -> Time:
        sec = int(ms / 1000.0)
        nanosec = int((ms / 1000.0 - sec) * 1e9)
        t = Time()
        t.sec = sec
        t.nanosec = nanosec
        return t

    def _to_image_msg(self, arr: np.ndarray, encoding: str, stamp: Time) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = arr.strides[0]
        msg.data = arr.tobytes()
        return msg

    def _build_camera_info(self, intr: rs.intrinsics) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.width = intr.width
        msg.height = intr.height
        msg.distortion_model = 'plumb_bob'
        msg.d = list(intr.coeffs)
        fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
