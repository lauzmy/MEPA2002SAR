#!/usr/bin/env python3
"""Subscribes to /thermal/pixel_raw, computes temperature stats, and publishes a colormapped image."""

# --- Imports ---
# third-party
import cv2
import numpy as np
from cv_bridge import CvBridge

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

RAW_TOPIC = '/thermal/pixel_raw'
HEAT_INFO_TOPIC = '/thermal/heat_info'
PROCESSED_IMAGE_TOPIC = '/thermal/processed_image'
INPUT_ENCODING = 'mono16'
OUTPUT_ENCODING = 'bgr8'

# Raw mono16 pixel = (centi-Kelvin); divide by 100 and shift by absolute zero to get Celsius.
TEMP_SCALE = 100.0
ABSOLUTE_ZERO_C = -273.15


class ThermalProcessing(Node):
    def __init__(self):
        super().__init__('thermal_processing')

        self.declare_parameter('display_min_c', 15.0)
        self.declare_parameter('display_max_c', 45.0)
        self._display_min_c = float(self.get_parameter('display_min_c').value)
        self._display_max_c = float(self.get_parameter('display_max_c').value)

        self._bridge = CvBridge()
        self._raw_sub = self.create_subscription(Image, RAW_TOPIC, self._on_thermal_frame, 10)
        self._heat_info_pub = self.create_publisher(String, HEAT_INFO_TOPIC, 10)
        self._image_pub = self.create_publisher(Image, PROCESSED_IMAGE_TOPIC, 10)

        self.get_logger().info(
            f'ThermalProcessing started: publishing {HEAT_INFO_TOPIC} and {PROCESSED_IMAGE_TOPIC}'
        )

    def _on_thermal_frame(self, msg):
        if self._heat_info_pub.get_subscription_count() == 0 and self._image_pub.get_subscription_count() == 0:
            return

        raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding=INPUT_ENCODING)
        temp_c = raw.astype(np.float32) / TEMP_SCALE + ABSOLUTE_ZERO_C

        if self._heat_info_pub.get_subscription_count() > 0:
            min_temp_c = float(np.min(temp_c))
            max_temp_c = float(np.max(temp_c))
            avg_temp_c = float(np.mean(temp_c))
            y, x = np.unravel_index(np.argmax(temp_c), temp_c.shape)
            self._heat_info_pub.publish(String(
                data=f'max={max_temp_c:.2f}, min={min_temp_c:.2f}, avg={avg_temp_c:.2f}, x={x}, y={y}'
            ))

        if self._image_pub.get_subscription_count() > 0:
            scaled = np.clip((temp_c - self._display_min_c) / (self._display_max_c - self._display_min_c), 0.0, 1.0)
            image_8bit = (scaled * 255).astype(np.uint8)
            colormap = cv2.applyColorMap(image_8bit, cv2.COLORMAP_INFERNO)
            image_msg = self._bridge.cv2_to_imgmsg(colormap, encoding=OUTPUT_ENCODING)
            image_msg.header = msg.header
            self._image_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalProcessing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
