#!/usr/bin/env python3
"""YOLO object detection on /camera/image_raw, publishes annotated frames on /yolo_coco/image."""

# --- Imports ---
# stdlib
import os
import time

# third-party
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

# project
from gregor_application.yolo_common import FpsLimiter, FpsTracker, draw_fps_overlay, ensure_ncnn

INPUT_TOPIC = '/camera/image_raw'
OUTPUT_TOPIC = '/yolo_coco/image'
ENCODING = 'bgr8'


class YoloCocoNode(Node):
    def __init__(self):
        super().__init__('yolo_coco_node')

        # --- Parameters ---
        default_model = os.path.join(get_package_share_directory('gregor_application'), 'yolo26n.pt')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('class_ids', [0])  # COCO 0 = person, 77 = teddy bear.
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('fps_limit', 0.0)  # 0.0 disables the limit.
        self.declare_parameter('cam_width', 0)    # 0 keeps the source width.
        self.declare_parameter('cam_height', 0)   # 0 keeps the source height.

        model_path = str(self.get_parameter('model_path').value)
        self._class_ids = list(self.get_parameter('class_ids').value)
        self._conf = float(self.get_parameter('conf').value)
        self._imgsz = int(self.get_parameter('imgsz').value)
        self._cam_width = int(self.get_parameter('cam_width').value)
        self._cam_height = int(self.get_parameter('cam_height').value)
        self._fps_limiter = FpsLimiter(float(self.get_parameter('fps_limit').value))

        # --- Model ---
        model_path = ensure_ncnn(model_path, self.get_logger())
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self._model = YOLO(model_path, task='detect')
        self._fps = FpsTracker(time.monotonic())

        # --- ROS interface ---
        self._bridge = CvBridge()
        self._image_sub = self.create_subscription(Image, INPUT_TOPIC, self._on_image, 10)
        self._image_pub = self.create_publisher(Image, OUTPUT_TOPIC, 10)

    def _on_image(self, msg):
        if self._image_pub.get_subscription_count() == 0:
            return

        now_s = time.monotonic()
        if self._fps_limiter.should_skip(now_s):
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding=ENCODING)
        if self._cam_width > 0 and self._cam_height > 0:
            frame = cv2.resize(frame, (self._cam_width, self._cam_height))

        results = self._model(frame, imgsz=self._imgsz, conf=self._conf,
                              classes=self._class_ids, verbose=False)
        annotated = results[0].plot()

        self._fps.update(now_s)
        draw_fps_overlay(annotated, self._fps.current_fps)

        self._image_pub.publish(self._bridge.cv2_to_imgmsg(annotated, encoding=ENCODING))


def main():
    rclpy.init()
    node = YoloCocoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
