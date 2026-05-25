#!/usr/bin/env python3
"""Reads raw mono16 frames from the thermal camera and publishes them on /thermal/pixel_raw."""

# --- Imports ---
# third-party
import cv2
import numpy as np
from cv_bridge import CvBridge

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

WIDTH = 160
HEIGHT = 120
RAW_TOPIC = '/thermal/pixel_raw'
ENCODING = 'mono16'
FOURCC_MONO16 = cv2.VideoWriter_fourcc('Y', '1', '6', ' ')
CAPTURE_PROPS = [
    (cv2.CAP_PROP_CONVERT_RGB, 0),
    (cv2.CAP_PROP_BUFFERSIZE, 1),
    (cv2.CAP_PROP_FOURCC, FOURCC_MONO16),
    (cv2.CAP_PROP_FRAME_WIDTH, WIDTH),
    (cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT),
]


def _to_mono16(frame):
    if frame is None:
        raise ValueError('Empty frame')
    if frame.ndim == 2 and frame.dtype == np.uint16:
        return frame
    # OpenCV hands back V4L2 Y16 as (H, W, 2) uint8 little-endian — reinterpret without copying.
    if frame.ndim == 3 and frame.shape[2] == 2 and frame.dtype == np.uint8:
        return frame.view(np.uint16).reshape(frame.shape[0], frame.shape[1])
    raise ValueError(f'Unsupported frame format: shape={frame.shape}, dtype={frame.dtype}')


class ThermalReading(Node):
    def __init__(self):
        super().__init__('thermal_reading')

        self.declare_parameter('device', '/dev/video2')
        self.declare_parameter('frame_id', 'thermal_camera')
        self.declare_parameter('fps', 9.0)

        device = str(self.get_parameter('device').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        fps = float(self.get_parameter('fps').value)

        self._bridge = CvBridge()
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        for prop, value in CAPTURE_PROPS:
            self._cap.set(prop, value)
        if not self._cap.isOpened():
            raise RuntimeError(f'Could not open camera: {device}')

        self._raw_pub = self.create_publisher(Image, RAW_TOPIC, 10)
        self._publish_timer = self.create_timer(1.0 / fps, self._publish_thermal_frame)
        self.get_logger().info(f'ThermalReading started on {device} -> {RAW_TOPIC} @ {fps} Hz')

    def _publish_thermal_frame(self):
        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warning('Failed to read frame')
            return

        raw = _to_mono16(frame)
        msg = self._bridge.cv2_to_imgmsg(raw, encoding=ENCODING)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._raw_pub.publish(msg)

    def destroy_node(self):
        self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThermalReading()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

