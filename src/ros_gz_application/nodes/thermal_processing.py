import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
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

        # Parameters
        self.declare_parameter('min_temp_celsius', 20.0)
        self.declare_parameter('max_temp_celsius', 100.0)
        self.declare_parameter('gaussian_blur_kernel', 3)
        self.declare_parameter('bad_pixel_correction', False)
        self.declare_parameter('hotspot_temp_celsius', 40.0)
        self.declare_parameter('hotspot_padding_px', 6)
        self.declare_parameter('hotspot_min_area_px', 20)

        # Read parameters
        self.min_temp = float(self.get_parameter('min_temp_celsius').value)
        self.max_temp = float(self.get_parameter('max_temp_celsius').value)
        self.blur_kernel = int(self.get_parameter('gaussian_blur_kernel').value)
        self.bad_pixel_correction = bool(self.get_parameter('bad_pixel_correction').value)
        self.hotspot_temp = float(self.get_parameter('hotspot_temp_celsius').value)
        self.hotspot_padding = int(self.get_parameter('hotspot_padding_px').value)
        self.hotspot_min_area = int(self.get_parameter('hotspot_min_area_px').value)
        self.heat_colormap = self._build_heat_colormap()

        # Subscribe to raw thermal topic from converter
        self.sub = self.create_subscription(Image, '/camera/raw_thermal', self.cb_image, 10)
        
        # Publishers
        self.temperature_pub = self.create_publisher(Image, '/thermal/temperature_image', 10)
        self.processed_pub = self.create_publisher(Image, '/thermal/processed_image', 10)
        self.avg_temp_pub = self.create_publisher(Float32, '/thermal/average_temperature', 10)
        self.max_temp_pub = self.create_publisher(Float32, '/thermal/max_temperature', 10)
        self.min_temp_pub = self.create_publisher(Float32, '/thermal/min_temperature', 10)

        self.get_logger().info('ThermalProcessor started')

    def cb_image(self, msg: Image):
        # Receive raw 16-bit mono thermal
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')

        # Convert raw 16-bit to Celsius
        raw_f = raw.astype(np.float32) / 65535.0
        temp = self.min_temp + raw_f * (self.max_temp - self.min_temp)

        # Pixel correction
        if self.bad_pixel_correction:
            temp = cv2.medianBlur(temp, 3)

        # Statistics
        valid_mask = np.isfinite(temp)
        if np.any(valid_mask):
            avg_temp = float(np.mean(temp[valid_mask]))
            max_temp = float(np.max(temp[valid_mask]))
            min_temp = float(np.min(temp[valid_mask]))
        else:
            avg_temp = max_temp = min_temp = 0.0

        self.avg_temp_pub.publish(Float32(data=avg_temp))
        self.max_temp_pub.publish(Float32(data=max_temp))
        self.min_temp_pub.publish(Float32(data=min_temp))

        # Publish per-pixel Celsius image as 32FC1.
        temp_msg = self.bridge.cv2_to_imgmsg(temp.astype(np.float32), encoding='32FC1')
        temp_msg.header = msg.header
        self.temperature_pub.publish(temp_msg)

        # Visualization: gray background with colored hot regions on top
        vis = np.clip((temp - self.min_temp) / (self.max_temp - self.min_temp), 0.0, 1.0)
        vis8 = (vis * 255.0).astype(np.uint8)
        if self.blur_kernel > 1:
            vis8 = cv2.GaussianBlur(vis8, (self.blur_kernel, self.blur_kernel), 0)

        vis_color = np.full((vis8.shape[0], vis8.shape[1], 3), 128, dtype=np.uint8)

        # Draw a red square around regions at or above the hotspot threshold.
        hotspot_mask = (temp >= self.hotspot_temp).astype(np.uint8)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(hotspot_mask, connectivity=8)

        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.hotspot_min_area:
                continue

            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            w = int(stats[label, cv2.CC_STAT_WIDTH])
            h = int(stats[label, cv2.CC_STAT_HEIGHT])

            x1 = max(0, x - self.hotspot_padding)
            y1 = max(0, y - self.hotspot_padding)
            x2 = min(vis_color.shape[1] - 1, x + w + self.hotspot_padding)
            y2 = min(vis_color.shape[0] - 1, y + h + self.hotspot_padding)

            component_mask = labels == label
            heat_vis = cv2.applyColorMap(vis8, self.heat_colormap)
            vis_color[component_mask] = heat_vis[component_mask]
            cv2.rectangle(vis_color, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Publish processed visualization
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
