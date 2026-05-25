#!/usr/bin/env python3
"""YOLO object detection on a GStreamer camera stream, publishes annotated frames on /yolo_coco/debug_image."""

# --- Imports ---
# stdlib
import os
import threading
import time

# third-party
import cv2
from ultralytics import YOLO

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

# project
from gregor_application.yolo_common import FpsLimiter, FpsTracker, draw_fps_overlay, ensure_ncnn

OUTPUT_TOPIC = '/yolo_coco/debug_image'
ENCODING = 'bgr8'
DEFAULT_PIPELINE = 'v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1'


class YoloGstreamerNode(Node):
    def __init__(self):
        super().__init__('yolo_gstreamer_node')

        # --- Parameters ---
        default_model = os.path.join(get_package_share_directory('gregor_application'), 'yolo26n.pt')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('class_ids', [77])  # COCO 0 = person, 77 = teddy bear.
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('fps_limit', 0.0)  # 0.0 disables the limit.
        self.declare_parameter('gstreamer_pipeline', DEFAULT_PIPELINE)

        model_path = str(self.get_parameter('model_path').value)
        self._class_ids = list(self.get_parameter('class_ids').value)
        self._conf = float(self.get_parameter('conf').value)
        self._imgsz = int(self.get_parameter('imgsz').value)
        self._pipeline = str(self.get_parameter('gstreamer_pipeline').value)
        self._fps_limiter = FpsLimiter(float(self.get_parameter('fps_limit').value))

        # --- Model ---
        model_path = ensure_ncnn(model_path, self.get_logger())
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self._model = YOLO(model_path, task='detect')
        self._fps = FpsTracker(time.monotonic())

        # --- ROS interface ---
        # Manual Image construction (no CvBridge) — sidesteps a numpy/cv_bridge ABI
        # crash on this deployment. See wiki: YOLO/CvBridgeWorkaround.
        self._image_pub = self.create_publisher(Image, OUTPUT_TOPIC, 10)

        # --- Worker thread ---
        # GStreamer + cv2.VideoCapture must not block the ROS executor; run it in a worker thread.
        self._running = True
        self._worker_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._worker_thread.start()

    def _camera_loop(self):
        self.get_logger().info(f'Opening GStreamer pipeline: {self._pipeline}')
        cap = cv2.VideoCapture(self._pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().error('Failed to open GStreamer video stream')
            return

        try:
            while self._running and rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warning('Lost frame from video stream, retrying...')
                    time.sleep(0.1)
                    continue

                if self._image_pub.get_subscription_count() == 0:
                    continue

                now_s = time.monotonic()
                if self._fps_limiter.should_skip(now_s):
                    continue

                results = self._model(frame, imgsz=self._imgsz, conf=self._conf,
                                      classes=self._class_ids, verbose=False)
                annotated = results[0].plot()

                self._fps.update(now_s)
                draw_fps_overlay(annotated, self._fps.current_fps)

                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                msg.height = annotated.shape[0]
                msg.width = annotated.shape[1]
                msg.encoding = ENCODING
                msg.is_bigendian = False
                msg.step = 3 * annotated.shape[1]
                msg.data = annotated.tobytes()
                self._image_pub.publish(msg)
        finally:
            cap.release()

    def destroy_node(self):
        self._running = False
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloGstreamerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
