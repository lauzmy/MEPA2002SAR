#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ThermalReading(Node):
    def __init__(self):
        super().__init__('thermal_reading')

        self.bridge = CvBridge()

        self.device = '/dev/video1'
        self.width = 160
        self.height = 120
        self.fps = 9.0
        #self.raw_display_min = 28315 #10 grader Celsius
        #self.raw_display_max = 33315 #60 grader Celsius

        self.raw_topic = '/thermal/pixel_raw'
        self.grey_topic = '/thermal/Image_raw'

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', '1', '6', ' '))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open camera: {self.device}')

        self.raw_pub = self.create_publisher(Image, self.raw_topic, 10)
        self.grey_pub = self.create_publisher(Image, self.grey_topic, 10)

        self.timer = self.create_timer(1.0 / self.fps, self.read_frame)

        self.get_logger().info(
            f'ThermalReading started: publishing {self.raw_topic} and {self.grey_topic}'
        )

    def read_frame(self):
        ok, frame = self.cap.read()

        if not ok:
            self.get_logger().warn('Failed to read frame')
            return

        raw = self.to_mono16(frame)

        stamp = self.get_clock().now().to_msg()

        raw_msg = self.bridge.cv2_to_imgmsg(raw, encoding='mono16')
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = 'thermal_camera'
        self.raw_pub.publish(raw_msg)

        grey = cv2.normalize(
            raw,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        ).astype(np.uint8)



        grey_msg = self.bridge.cv2_to_imgmsg(grey, encoding='mono8')
        grey_msg.header.stamp = stamp
        grey_msg.header.frame_id = 'thermal_camera'
        self.grey_pub.publish(grey_msg)

    def to_mono16(self, frame):
        if frame is None:
            raise ValueError('Empty frame')

        if frame.ndim == 2 and frame.dtype == np.uint16:
            return frame

        if frame.ndim == 3 and frame.shape[2] == 2 and frame.dtype == np.uint8:
            return frame[:, :, 0].astype(np.uint16) | (
                frame[:, :, 1].astype(np.uint16) << 8
            )

        raise ValueError(
            f'Unsupported frame format: shape={frame.shape}, dtype={frame.dtype}'
        )

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ThermalReading()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

