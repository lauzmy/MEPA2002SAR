#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ThermalConverter(Node):
    """Read raw thermal camera and publish as ROS Image topic"""

    def __init__(self):
        super().__init__('thermal_converter')
        self.bridge = CvBridge()
        self.frame_count = 0

        #videoframe configuration
        self.video_device = '/dev/video2'
        self.frame_rate = 9.0
        self.output_topic = '/camera/raw_thermal'
        self.force_y16 = True
        self.frame_width = 160
        self.frame_height = 120
        self.show_preview = True
        self.preview_scale = 4.0
        self.preview_window_name = 'PureThermal3 Raw'

        # Open camera
        self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        
        if self.frame_width > 0 and self.frame_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        
        if self.force_y16:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', '1', '6', ' '))
        
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open video device: {self.video_device}')

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = ''.join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))

        # Publishers: publish raw thermal data
        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.frame_rate, self.cb_capture)

        self.get_logger().info(
            f'ThermalConverter started: device={self.video_device}, '
            f'format={fourcc_str.strip() or fourcc}, '
            f'size={actual_w}x{actual_h}, preview={self.show_preview}'
        )

    def cb_capture(self):
        ok, frame = self.cap.read()
        
        if not ok:
            raise RuntimeError(f'Failed to capture frame from {self.video_device}')

        self.frame_count += 1
        raw = self.to_mono16(frame)

        # Publish raw 16-bit image (no conversion to Celsius here)
        out_msg = self.bridge.cv2_to_imgmsg(raw.astype(np.uint16), encoding='mono16')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'thermal_camera'
        self.pub.publish(out_msg)

        # Optional preview
        if self.show_preview:
            self._show_preview(raw)

        # Log first frame
        if self.frame_count == 1:
            self.get_logger().info(
                f'First frame: {raw.shape[1]}x{raw.shape[0]}, {raw.dtype}'
            )

    def _show_preview(self, raw):
        """Display raw thermal with false colors (for debug only)"""
        display_8u = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        colored = cv2.applyColorMap(display_8u, cv2.COLORMAP_INFERNO)

        if self.preview_scale != 1.0:
            width = int(colored.shape[1] * self.preview_scale)
            height = int(colored.shape[0] * self.preview_scale)
            colored = cv2.resize(colored, (width, height), interpolation=cv2.INTER_NEAREST)
        
        cv2.imshow(self.preview_window_name, colored)
        key = cv2.waitKey(1) & 0xFF
        
        if key == 255:
            pass  # No key
        elif key == ord('q') or key == 27:  # q or Esc
            self.get_logger().info('User pressed quit, shutting down')
            rclpy.shutdown()
        else:
            self.get_logger().debug(f'Key: {key}')

    def to_mono16(self, frame):
        """Normalize frame to uint16 mono"""
        if frame is None:
            raise ValueError('Empty frame')

        # Fast path: already uint16 mono (expected for PureThermal3 Y16)
        if frame.ndim == 2 and frame.dtype == np.uint16:
            return frame
        
        # Handle edge cases
        if frame.ndim == 3:
            if frame.shape[2] == 2 and frame.dtype == np.uint8:
                # 2-channel uint8: little-endian uint16
                return frame[:, :, 0].astype(np.uint16) | (frame[:, :, 1].astype(np.uint16) << 8)
            else:
                # Convert multi-channel to grayscale
                color_code = cv2.COLOR_BGR2GRAY if frame.shape[2] == 3 else cv2.COLOR_BGRA2GRAY
                frame = cv2.cvtColor(frame, color_code)
        
        # Convert to uint16 if not already
        if frame.dtype == np.uint16:
            return frame
        elif frame.dtype == np.uint8:
            return frame.astype(np.uint16) * 257
        else:
            raise ValueError(f'Unsupported dtype: {frame.dtype}')

    def destroy_node(self):
        if self.show_preview:
            cv2.destroyWindow(self.preview_window_name)
        
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThermalConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

