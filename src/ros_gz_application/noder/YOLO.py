#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from ament_index_python.packages import get_package_share_directory


class YoloCocoNode(Node):
    def __init__(self):
        super().__init__("yolo_coco_node")
        pkg_dir = get_package_share_directory('ros_gz_application')
        default_model = os.path.join(pkg_dir, 'yolo26n-seg.pt')

        self.declare_parameter("model_path", default_model)
        self.declare_parameter("class_ids", [0]) # 0 is person in COCO, teddy bear is 77
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)

        model_path = self.get_parameter("model_path").value
        self.class_ids = set(self.get_parameter("class_ids").value)
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)

        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_cb, 10
        )
        self.pub = self.create_publisher(Image, "/yolo_coco/image", 10)

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Vi sender klassene vi vil ha direkte inn i søket (classes=list(self.class_ids))
        results = self.model(frame, imgsz=self.imgsz, conf=self.conf, classes=list(self.class_ids), verbose=False)

        # results[0].plot() tegner automatisk inn bounding boxes, labels, confidence OG masker!
        if len(results) > 0:
            annotated_frame = results[0].plot()
        else:
            annotated_frame = frame

        # Konverter tilbake til ROS Image og publiser
        out = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.pub.publish(out)

def main():
    rclpy.init()
    node = YoloCocoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()