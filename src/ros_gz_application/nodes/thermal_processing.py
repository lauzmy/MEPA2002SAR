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

    @staticmethod
    def _build_heat_colormap():
        anchors = np.array(
            [
                [0, 0, 255],      # blue
                [130, 0, 180],    # purple
                [255, 0, 255],    # pink
                [255, 0, 0],      # red
                [255, 140, 0],    # orange
                [255, 255, 0],    # yellow
            ],
            dtype=np.float32,
        )
        positions = np.array([0, 51, 102, 153, 204, 255], dtype=np.float32)
        lut = np.zeros((256, 1, 3), dtype=np.uint8)

        for i in range(256):
            idx = np.searchsorted(positions, i, side='right')
            if idx <= 0:
                color = anchors[0]
            elif idx >= len(anchors):
                color = anchors[-1]
            else:
                left_pos = positions[idx - 1]
                right_pos = positions[idx]
                alpha = (i - left_pos) / max(1.0, right_pos - left_pos)
                color = (1.0 - alpha) * anchors[idx - 1] + alpha * anchors[idx]
            lut[i, 0] = np.clip(color, 0, 255).astype(np.uint8)

        return lut

    def __init__(self):
        super().__init__('thermal_processor')
        self.bridge = CvBridge()

        #temperature cofiguration
        self.min_temp = 20.0
        self.max_temp = 100.0
        self.blur_kernel = 3
        self.bad_pixel_correction = False
        self.hotspot_temp = 40.0
        self.hotspot_padding = 6
        self.hotspot_min_area = 20
        self.heat_colormap = self._build_heat_colormap()

        # Subscribe to raw thermal topic from converter
        self.sub = self.create_subscription(Image, '/camera/raw_thermal', self.cb_image, 10)

        # Publishers
        self.heat_info_pub = self.create_publisher(String, '/thermal/heat_info', 10)
        self.processed_pub = self.create_publisher(Image, '/thermal/processed_image', 10)

        self.get_logger().info('ThermalProcessor started')

    def cb_image(self, msg: Image):
        # Receive raw 16-bit mono thermal
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')

        # Convert raw 16-bit to Celsius
        temp = raw.astype(np.float32) / 100.0 - 273.15

        # Pixel correction
        if self.bad_pixel_correction:
            temp = cv2.medianBlur(temp, 3)

        # Warm area ratio in the configured heat range (20-100 C).
        warm_mask = (temp >= 20.0) & (temp <= 100.0)
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

        in_heat_range = 20.0 <= hottest_temp <= 100.0
        status = 'heat_detected' if in_heat_range else 'no_heat'
        info_msg = String()
        info_msg.data = (
            f'status={status}, temp_c={hottest_temp:.1f}, '
            f'coordinates=({x_hottest},{y_hottest}), '
            f'proximity={proximity}, warm_ratio={warm_ratio:.3f}'
        )
        self.heat_info_pub.publish(info_msg)

        # Build a gray background with colormap only on warm regions.
        vis = np.clip((temp - self.min_temp) / (self.max_temp - self.min_temp), 0.0, 1.0)
        vis8 = (vis * 255.0).astype(np.uint8)
        if self.blur_kernel > 1:
            vis8 = cv2.GaussianBlur(vis8, (self.blur_kernel, self.blur_kernel), 0)

        vis_color = np.full((vis8.shape[0], vis8.shape[1], 3), 128, dtype=np.uint8)
        heat_vis = cv2.applyColorMap(vis8, self.heat_colormap)
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
