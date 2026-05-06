import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32
from cv_bridge import CvBridge
import numpy as np
import cv2


class ThermalProcessor(Node):
    """Subscribe to raw thermal, convert to Celsius, detect hotspots, classify by temperature"""

    def __init__(self):
        super().__init__('thermal_processor')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('min_temp_celsius', 20.0)
        self.declare_parameter('max_temp_celsius', 100.0)
        self.declare_parameter('temp_bins', [20.0, 35.0, 40.0, 50.0, 60.0])
        self.declare_parameter('num_top_spots', 3)
        self.declare_parameter('min_area_pixels', 50)
        self.declare_parameter('gaussian_blur_kernel', 3)
        self.declare_parameter('bad_pixel_correction', False)

        # Read parameters
        self.min_temp = float(self.get_parameter('min_temp_celsius').value)
        self.max_temp = float(self.get_parameter('max_temp_celsius').value)
        self.temp_bins = sorted([float(b) for b in self.get_parameter('temp_bins').value])
        self.num_top_spots = int(self.get_parameter('num_top_spots').value)
        self.min_area = int(self.get_parameter('min_area_pixels').value)
        self.blur_kernel = int(self.get_parameter('gaussian_blur_kernel').value)
        self.bad_pixel_correction = bool(self.get_parameter('bad_pixel_correction').value)

        # Subscribe to raw thermal topic from converter
        self.sub = self.create_subscription(Image, '/camera/raw_thermal', self.cb_image, 10)
        
        # Publishers
        self.processed_pub = self.create_publisher(Image, '/thermal/processed_image', 10)
        self.top_spots_pub = self.create_publisher(Float32MultiArray, '/thermal/top_spots', 10)
        self.avg_temp_pub = self.create_publisher(Float32, '/thermal/average_temperature', 10)
        self.max_temp_pub = self.create_publisher(Float32, '/thermal/max_temperature', 10)
        self.min_temp_pub = self.create_publisher(Float32, '/thermal/min_temperature', 10)

        # Per-bin publishers
        bins = self.temp_bins
        labels = []
        labels.append(f'below_{int(bins[0])}')
        for i in range(len(bins) - 1):
            labels.append(f'{int(bins[i])}_{int(bins[i+1])}')
        labels.append(f'above_{int(bins[-1])}')
        self.bin_pubs = []
        for idx, label in enumerate(labels):
            topic = f'/thermal/bin_{idx}_{label}_spots'
            self.bin_pubs.append(self.create_publisher(Float32MultiArray, topic, 10))

        self.get_logger().info('ThermalProcessor started')

    def _classify_temperature(self, t):
        """Classify temperature into bin index"""
        if t < self.temp_bins[0]:
            return 0
        for i in range(len(self.temp_bins) - 1):
            if t < self.temp_bins[i + 1]:
                return i + 1
        return len(self.temp_bins)

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

        # Visualization: normalize and apply colormap
        vis = np.clip((temp - self.min_temp) / (self.max_temp - self.min_temp), 0.0, 1.0)
        vis8 = (vis * 255.0).astype(np.uint8)
        if self.blur_kernel > 1:
            vis8 = cv2.GaussianBlur(vis8, (self.blur_kernel, self.blur_kernel), 0)

        # Hotspot detection: find pixels above lowest bin threshold
        detect_mask = temp >= self.temp_bins[0]
        mask_uint8 = detect_mask.astype(np.uint8) * 255
        contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        spots = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            cx = max(0, min(cx, temp.shape[1] - 1))
            cy = max(0, min(cy, temp.shape[0] - 1))
            t = float(temp[cy, cx])
            spots.append((cx, cy, t))

        # Classify into temperature bins
        classified = {}
        for x, y, t in spots:
            bin_idx = self._classify_temperature(t)
            classified.setdefault(bin_idx, []).append((x, y, t))

        # Publish per-bin data
        for idx, pub in enumerate(self.bin_pubs):
            bin_list = classified.get(idx, [])
            arr = Float32MultiArray()
            arr.data = []
            for x, y, t in bin_list:
                arr.data.extend([float(x), float(y), float(t)])
            pub.publish(arr)

        # Publish top-N hottest spots
        top_spots = sorted(spots, key=lambda s: s[2], reverse=True)[: self.num_top_spots]
        arr = Float32MultiArray()
        arr.data = []
        for x, y, t in top_spots:
            arr.data.extend([float(x), float(y), float(t)])
        self.top_spots_pub.publish(arr)

        # Visualization: mark top spots on colormap
        vis_color = cv2.applyColorMap(vis8, cv2.COLORMAP_JET)
        for x, y, t in top_spots:
            cv2.circle(vis_color, (int(x), int(y)), 5, (0, 0, 255), 2)

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
