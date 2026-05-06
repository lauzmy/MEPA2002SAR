#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ThermalConverter(Node):
    """Converts raw thermal images (16-bit mono) to Celsius (32-bit float)"""

    def __init__(self):
        super().__init__('thermal_converter')
        self.bridge = CvBridge()
        self.frame_count = 0

        # Parameters
        self.declare_parameter('min_temp_celsius', 20.0)
        self.declare_parameter('max_temp_celsius', 100.0)
        self.declare_parameter('bad_pixel_correction', False)
        self.declare_parameter('video_device', '/dev/video2')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('output_topic', '/camera/thermal_celsius')
        self.declare_parameter('force_y16', True)
        self.declare_parameter('frame_width', 160)
        self.declare_parameter('frame_height', 120)
        self.declare_parameter('show_preview', True)
        self.declare_parameter('preview_window_name', 'PureThermal3 Preview')

        # Load parameters
        self.min_temp = float(self.get_parameter('min_temp_celsius').value)
        self.max_temp = float(self.get_parameter('max_temp_celsius').value)
        self.bad_pixel_correction = bool(self.get_parameter('bad_pixel_correction').value)
        self.video_device = str(self.get_parameter('video_device').value)
        self.frame_rate = max(1.0, float(self.get_parameter('frame_rate').value))
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.force_y16 = bool(self.get_parameter('force_y16').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.preview_window_name = str(self.get_parameter('preview_window_name').value)

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

        # Publishers
        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.frame_rate, self.cb_capture)

        self.get_logger().info(
            f'ThermalConverter started: device={self.video_device}, '
            f'temp={self.min_temp}-{self.max_temp}C, format={fourcc_str.strip() or fourcc}, '
            f'size={actual_w}x{actual_h}, preview={self.show_preview}'
        )

    def cb_capture(self):
        ok, frame = self.cap.read()
        
        if not ok:
            raise RuntimeError(f'Failed to capture frame from {self.video_device}')

        self.frame_count += 1
        raw = self.to_mono16(frame)

        # Convert to float32 0..1
        raw_f = raw.astype(np.float32) / 65535.0

        # Map to Celsius range
        temp_c = self.min_temp + raw_f * (self.max_temp - self.min_temp)

        # Pixel correction
        if self.bad_pixel_correction:
            temp_c = cv2.medianBlur(temp_c, 3)

        # Publish Celsius image
        out_msg = self.bridge.cv2_to_imgmsg(temp_c.astype(np.float32), encoding='32FC1')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'thermal_camera'
        self.pub.publish(out_msg)

        # Preview or skip
        if self.show_preview:
            self._show_preview(temp_c)
        else:
            self.get_logger().debug('Preview disabled')

        # Log first frame
        if self.frame_count == 1:
            self.get_logger().info(
                f'First frame: {raw.shape[1]}x{raw.shape[0]}, {raw.dtype}'
            )

    def _show_preview(self, temp_c):
        """Display thermal preview with false colors"""
        display_8u = cv2.normalize(temp_c, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        preview = cv2.applyColorMap(display_8u, cv2.COLORMAP_INFERNO)
        
        cv2.imshow(self.preview_window_name, preview)
        key = cv2.waitKey(1) & 0xFF
        
        if key == 255:
            pass  # No key
        elif key == ord('q'):
            self.get_logger().info('User pressed q, shutting down')
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

