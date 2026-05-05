import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class ThermalConverter(Node):
    
    "Konverterer rå termiske bilder (16-bit mono) til Celsius (32-bit float) basert på kalibrering"
    
    def __init__(self):
        super().__init__('thermal_converter')
        self.bridge = CvBridge()

        # Parametere for kalibrereing av termiske bilder
        self.declare_parameter('min_temp_celsius', 20.0)
        self.declare_parameter('max_temp_celsius', 100.0)
        self.declare_parameter('bad_pixel_correction', False)

        self.min_temp = float(self.get_parameter('min_temp_celsius').value)
        self.max_temp = float(self.get_parameter('max_temp_celsius').value)
        self.bad_pixel_correction = bool(self.get_parameter('bad_pixel_correction').value)

        # Sub / Pub
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb_image, 10)
        self.pub = self.create_publisher(Image, '/camera/thermal_celsius', 10)

        self.get_logger().info(f'ThermalConverter started (min={self.min_temp}C max={self.max_temp}C)')

    def cb_image(self, msg: Image):
        try:
            # Expecting 16-bit thermal image (mono16 / L16)
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')

            # Konverter til float32 0..1
            raw_f = raw.astype(np.float32) / 65535.0

            # Mapper celsius til min/max
            temp_c = self.min_temp + raw_f * (self.max_temp - self.min_temp)

            # Pixel Korrigerering ved behov
            if self.bad_pixel_correction:
                # Median filter for å  pikselkorrigering
                import cv2
                temp_c = cv2.medianBlur(temp_c, 3)

            # Publiserer bildet som 32-bit float med Celsius-verdier
            out_msg = self.bridge.cv2_to_imgmsg(temp_c.astype(np.float32), encoding='32FC1')
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Converter error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ThermalConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

