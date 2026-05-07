#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2


class ThermalProcessor(Node):
    """Subscribe to raw thermal, convert to Celsius, and visualize warm regions."""
    
    def __init__(self):
        super().__init__('thermal_processor')
        self.bridge = CvBridge()

        #temperature configuration
        # Full sensor temperature range shown in visualization
        self.min_temp = 0.0
        # Upper bound for inferno colormap (tuneable: 60 or 70 are common)
        self.max_temp = 60.0
        # Temperature threshold where we switch from grayscale -> inferno
        self.inferno_threshold = 25.0
        self.blur_kernel = 3
        self.bad_pixel_correction = False
        self.hotspot_temp = 40.0

        self.hotspot_padding = 6
        self.hotspot_min_area = 20

        # Calibration offset (deg C) to correct systematic bias in readings
        # Set e.g. -10.0 if temps are consistently 10°C too high
        self.temp_offset = 0.0

        # How many degrees below `min_temp` will be shown as a grayscale ramp
        # How many degrees below `inferno_threshold` will be shown as a grayscale ramp
        # For grayscale 0..25 set gray_span == inferno_threshold
        self.gray_span = float(self.inferno_threshold)

        # Use OpenCV inferno colormap for thermal visualization
        self.heat_colormap = cv2.COLORMAP_INFERNO

        # Subscribe to raw thermal topic from converter
        self.sub = self.create_subscription(Image, '/camera/raw_thermal', self.cb_image, 10)

        # Publishers
        self.heat_info_pub = self.create_publisher(String, '/thermal/heat_info', 10)
        self.processed_pub = self.create_publisher(Image, '/thermal/processed_image', 10)

        self.get_logger().info('ThermalProcessor started')

    def cb_image(self, msg: Image):
        # Receive raw 16-bit mono thermal
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')

        # Convert raw 16-bit to Celsius and apply optional calibration offset
        temp = raw.astype(np.float32) / 100.0 - 273.15 + float(self.temp_offset)

        # Pixel correction
        if self.bad_pixel_correction:
            temp = cv2.medianBlur(temp, 3)

        # Warm area ratio in the inferno range (inferno_threshold .. max_temp).
        warm_mask = (temp >= self.inferno_threshold) & (temp <= self.max_temp)
        warm_ratio = float(np.count_nonzero(warm_mask)) / float(warm_mask.size)

        if warm_ratio < 0.05:
            proximity = 'far'
        elif warm_ratio < 0.20:
            proximity = 'medium'
        else:
            proximity = 'close'

        # Find hottest spot
        valid_mask = np.isfinite(temp)
        hottest_temp = 0.0
        x_hottest = -1
        y_hottest = -1
        if np.any(valid_mask):
            # Find coordinates and value of hottest pixel
            max_idx = np.argmax(temp)
            y_hottest, x_hottest = np.unravel_index(max_idx, temp.shape)
            hottest_temp = float(temp[y_hottest, x_hottest])

        in_heat_range = self.inferno_threshold <= hottest_temp <= self.max_temp
        status = 'heat_detected' if in_heat_range else 'no_heat'
        info_msg = String()
        info_msg.data = (
            f'status={status}, temp_c={hottest_temp:.1f}, '
            f'coordinates=({x_hottest},{y_hottest}), '
            f'proximity={proximity}, warm_ratio={warm_ratio:.3f}'
        )
        self.heat_info_pub.publish(info_msg)

        # Build a grayscale background for low temperatures and apply
        # an inferno colormap only on warm regions.
        # For inferno color mapping normalize over [inferno_threshold .. max_temp]
        denom = max(1e-6, (self.max_temp - self.inferno_threshold))
        vis = np.clip((temp - self.inferno_threshold) / denom, 0.0, 1.0)
        vis8 = (vis * 255.0).astype(np.uint8)
        if self.blur_kernel > 1:
            vis8 = cv2.GaussianBlur(vis8, (self.blur_kernel, self.blur_kernel), 0)

        # Grayscale ramp below `inferno_threshold` (from 0 .. inferno_threshold)
        gray_min = self.min_temp
        gray_denom = max(1e-6, (self.inferno_threshold - gray_min))
        gray_vis = np.clip((temp - gray_min) / gray_denom, 0.0, 1.0)
        gray8 = (gray_vis * 255.0).astype(np.uint8)
        vis_color = cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)

        # Colorize warm regions using inferno
        heat_vis = cv2.applyColorMap(vis8, int(self.heat_colormap))
        vis_color[warm_mask] = heat_vis[warm_mask]

        out_msg = self.bridge.cv2_to_imgmsg(vis_color, encoding='bgr8')
        out_msg.header = msg.header
        self.processed_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
