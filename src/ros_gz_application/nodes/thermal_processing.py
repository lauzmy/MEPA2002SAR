#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2


class ThermalProcessing(Node):

    def __init__(self):
        super().__init__('thermal_processing')

        self.bridge = CvBridge()
        self.temp_scale = 100.0
        self.temp_offset = -273.15

        self.sub = self.create_subscription(
            Image,
            '/thermal/pixel_raw',
            self.process_frame,
            10
        )

        self.heat_info_pub = self.create_publisher(
            String,
            '/thermal/heat_info',
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/thermal/processed_image',
            10
        )

        self.get_logger().info(
            'ThermalProcessing started: publishing /thermal/heat_info and /thermal/processed_image'
        )

    def process_frame(self, msg):
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')

        # raw pixels -> Celsius
        temp = raw.astype(np.float32) / self.temp_scale + self.temp_offset

        # temperature stats
        min_temp = float(np.min(temp))
        max_temp = float(np.max(temp))
        avg_temp = float(np.mean(temp))

        # hottest pixel coordinate
        y, x = np.unravel_index(np.argmax(temp), temp.shape)

        # publish heat info
        info = String()
        info.data = (
            f'max={max_temp:.2f}, '
            f'min={min_temp:.2f}, '
            f'avg={avg_temp:.2f}, '
            f'x={x}, '
            f'y={y}'
        )
        self.heat_info_pub.publish(info)

        # make inferno colormap image
        image_8bit = cv2.normalize(
            temp,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        ).astype(np.uint8)

        colormap = cv2.applyColorMap(
            image_8bit,
            cv2.COLORMAP_INFERNO
        )

        # publish processed image for RViz
        image_msg = self.bridge.cv2_to_imgmsg(
            colormap,
            encoding='bgr8'
        )
        image_msg.header = msg.header

        self.image_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalProcessing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()