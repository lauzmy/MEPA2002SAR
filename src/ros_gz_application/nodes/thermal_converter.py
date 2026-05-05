#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ThermalConverter(Node):
    
    "Konverterer rå termiske bilder (16-bit mono) til Celsius (32-bit float) basert på kalibrering"
    
    def __init__(self):
        super().__init__('thermal_converter')
        self.bridge = CvBridge()
        self.frame_count = 0
        self.read_failures = 0

        # Parametere for kalibrering av termiske bilder
        self.declare_parameter('min_temp_celsius', 20.0)
        self.declare_parameter('max_temp_celsius', 100.0)
        self.declare_parameter('bad_pixel_correction', False)
        self.declare_parameter('video_device', '/dev/video2')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('output_topic', '/camera/thermal_celsius')
        self.declare_parameter('force_y16', True)
        self.declare_parameter('frame_width', 160)
        self.declare_parameter('frame_height', 120)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('preview_window_name', 'PureThermal3 Preview')

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

        # Les direkte fra V4L2-enheten (f.eks. /dev/video2)
        self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        if self.frame_width > 0 and self.frame_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if self.force_y16:
            # PureThermal3 gir ofte 16-bit rå termiske verdier via Y16 over V4L2.
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', '1', '6', ' '))
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open video device: {self.video_device}')

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = ''.join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))

        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.frame_rate, self.cb_capture)

        self.get_logger().info(
            f'ThermalConverter started (device={self.video_device}, '
            f'min={self.min_temp}C max={self.max_temp}C, out={self.output_topic}, '
            f'format={fourcc_str.strip() or fourcc}, size={actual_w}x{actual_h})'
        )

    def cb_capture(self):
        try:
            ok, frame = self.cap.read()
            if not ok:
                self.read_failures += 1
                if self.read_failures % 30 == 1:
                    self.get_logger().warning(f'No frame from {self.video_device} yet')
                return

            self.frame_count += 1
            raw = self.to_mono16(frame)

            # Konverter til float32 0..1
            raw_f = raw.astype(np.float32) / 65535.0

            # Mapper celsius til min/max
            temp_c = self.min_temp + raw_f * (self.max_temp - self.min_temp)

            # Pixel Korrigerering ved behov
            if self.bad_pixel_correction:
                # Median filter for enkel pikselkorrigering
                temp_c = cv2.medianBlur(temp_c, 3)

            # Publiserer bildet som 32-bit float med Celsius-verdier
            out_msg = self.bridge.cv2_to_imgmsg(temp_c.astype(np.float32), encoding='32FC1')
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = 'thermal_camera'
            self.pub.publish(out_msg)

            if self.show_preview:
                # Skaler Celsius-bildet til visning og legg på falske farger for enkel inspeksjon.
                display_8u = cv2.normalize(temp_c, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                preview = cv2.applyColorMap(display_8u, cv2.COLORMAP_INFERNO)
                cv2.imshow(self.preview_window_name, preview)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('Preview closed by user (q), shutting down node')
                    rclpy.shutdown()

            if self.frame_count == 1:
                self.get_logger().info(
                    f'First frame received from {self.video_device} ({raw.shape[1]}x{raw.shape[0]})'
                )

        except Exception as e:
            self.get_logger().error(f'Converter error: {e}')

    def to_mono16(self, frame):
        # Kamera kan levere ulike formater; normaliser til 16-bit mono for videre konvertering.
        if frame is None:
            raise ValueError('Empty frame from camera')

        if frame.ndim == 2:
            if frame.dtype == np.uint16:
                return frame
            if frame.dtype == np.uint8:
                return frame.astype(np.uint16) * 257
            return np.clip(frame, 0, 65535).astype(np.uint16)

        if frame.ndim == 3:
            if frame.shape[2] == 2 and frame.dtype == np.uint8:
                # Enkelte V4L2/OpenCV-kombinasjoner leverer Y16 som 2x8-bit kanaler.
                return frame[:, :, 0].astype(np.uint16) | (frame[:, :, 1].astype(np.uint16) << 8)
            if frame.shape[2] == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                return gray.astype(np.uint16) * 257
            if frame.shape[2] == 4:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
                return gray.astype(np.uint16) * 257

        raise ValueError(f'Unsupported frame format: shape={frame.shape}, dtype={frame.dtype}')

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

