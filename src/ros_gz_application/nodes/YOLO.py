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
        
        # Standar .pt model
        default_pt_model = os.path.join(pkg_dir, 'yolo26n.pt')

        self.declare_parameter("model_path", default_pt_model)
        self.declare_parameter("class_ids", [0]) # 0 is person in COCO, teddy bear is 77
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)
        
        # Nye parametere for FPS og oppløsning
        self.declare_parameter("fps_limit", 0.0)    # 0.0 = ingen begrensning
        self.declare_parameter("cam_width", 0)      # 0 = bruk original bredde
        self.declare_parameter("cam_height", 0)     # 0 = bruk original høyde

        # Get parameter values
        model_path = self.get_parameter("model_path").value
        self.class_ids = set(self.get_parameter("class_ids").value)
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        
        self.fps_limit = float(self.get_parameter("fps_limit").value)
        self.cam_width = int(self.get_parameter("cam_width").value)
        self.cam_height = int(self.get_parameter("cam_height").value)

        # Variabler for å regne ut og begrense FPS
        self.last_frame_time = self.get_clock().now()
        self.fps_calc_time = self.get_clock().now()
        self.frames_processed = 0
        self.current_fps = 0.0

        # ---- NCNN INTEGRATION ----
        # If pointing to a .pt-file, switch to ncnn
        if model_path.endswith('.pt'):
            ncnn_model_dir = model_path.replace('.pt', '_ncnn_model')
            
            # Check if NCNN folder is missing
            if not os.path.exists(ncnn_model_dir):
                self.get_logger().info(f"Could not find NCNN model. Exporting {model_path} to NCNN. This may take a while...")
                temp_model = YOLO(model_path, task="detect")
                temp_model.export(format="ncnn")
                self.get_logger().info(f"Export complete! Saved to {ncnn_model_dir}")
            
            # NCNN exists, translate model_path to ncnn directory
            model_path = ncnn_model_dir

        self.get_logger().info(f"Loading model from: {model_path}")
        # Loading the YOLO model
        self.model = YOLO(model_path, task="detect")

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_cb, 10
        )
        self.pub = self.create_publisher(Image, "/yolo_coco/image", 10)

    def image_cb(self, msg):
        now = self.get_clock().now()

        # 1. Sjekk om vi skal hoppe over bildet for å overholde FPS-begrensningen
        if self.fps_limit > 0.0:
            elapsed_time = (now - self.last_frame_time).nanoseconds / 1e9
            if elapsed_time < (1.0 / self.fps_limit):
                return  # Dropper bildet / capper FPS

        self.last_frame_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # 2. Scaler bildet hvis brukeren har satt en oppløsning
        if self.cam_width > 0 and self.cam_height > 0:
            frame = cv2.resize(frame, (self.cam_width, self.cam_height))
        
        # 3. Kjør inference
        results = self.model(frame, imgsz=self.imgsz, conf=self.conf, classes=list(self.class_ids), verbose=False)

        if len(results) > 0:
            annotated_frame = results[0].plot()
        else:
            annotated_frame = frame

        # 4. Regn ut og vis FPS
        self.frames_processed += 1
        fps_time_elapsed = (now - self.fps_calc_time).nanoseconds / 1e9
        
        if fps_time_elapsed >= 1.0:  # Oppdater FPS-telleren hvert sekund
            self.current_fps = self.frames_processed / fps_time_elapsed
            self.frames_processed = 0
            self.fps_calc_time = now
            
        # Tegn FPS oppe i venstre hjørne (Tekst, posisjon, font, størrelse, farge, tykkelse)
        cv2.putText(
            annotated_frame, 
            f"FPS: {self.current_fps:.1f}", 
            (10, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, 
            (0, 255, 0), 
            2
        )

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