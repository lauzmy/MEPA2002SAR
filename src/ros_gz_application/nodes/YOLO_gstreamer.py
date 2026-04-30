#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
import threading
import time
from ament_index_python.packages import get_package_share_directory

class YoloGstreamerNode(Node):
    def __init__(self):
        super().__init__("yolo_gstreamer_node")
        pkg_dir = get_package_share_directory('ros_gz_application')
        default_pt_model = os.path.join(pkg_dir, 'yolo26n.pt')

        # Oppsett av parametere
        self.declare_parameter("model_path", default_pt_model)
        self.declare_parameter("class_ids", [0])
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("fps_limit", 0.0)
        
        # GStreamer pipeline for å hente video (Standard er USB-kamera på Linux)
        default_pipeline = "v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1"
        self.declare_parameter("gstreamer_pipeline", default_pipeline)

        # Hent parametere
        model_path = self.get_parameter("model_path").value
        self.class_ids = set(self.get_parameter("class_ids").value)
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.fps_limit = float(self.get_parameter("fps_limit").value)
        self.pipeline = self.get_parameter("gstreamer_pipeline").value

        # NCNN Eksport-logikk
        if model_path.endswith('.pt'):
            ncnn_model_dir = model_path.replace('.pt', '_ncnn_model')
            if not os.path.exists(ncnn_model_dir):
                self.get_logger().info(f"Eksporterer {model_path} til NCNN...")
                temp_model = YOLO(model_path, task="detect")
                temp_model.export(format="ncnn")
                self.get_logger().info(f"Eksport ferdig! Lagret i {ncnn_model_dir}")
            model_path = ncnn_model_dir

        self.get_logger().info(f"Laster modell fra: {model_path}")
        self.model = YOLO(model_path, task="detect")
        self.bridge = CvBridge()
        
        # Vi publiserer fremdeles debug-bildet til ROS hvis ønskelig
        self.pub = self.create_publisher(Image, "/yolo_coco/debug_image", 10)

        self.frames_processed = 0
        self.current_fps = 0.0
        self.fps_calc_time = time.time()

        # Start en dedikert tråd for å lese fra kameraet (forhindrer at ROS henger seg)
        self.running = True
        self.worker_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.worker_thread.start()

    def camera_loop(self):
        self.get_logger().info(f"Åpner GStreamer-pipeline: {self.pipeline}")
        cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            self.get_logger().error("Klarte ikke å åpne GStreamer videostrøm!")
            return

        last_frame_time = time.time()

        while self.running and rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn("Mistet bilde fra videostrøm! Prøver igjen...")
                time.sleep(0.1)
                continue

            now = time.time()
            
            # FPS capping
            if self.fps_limit > 0.0:
                if (now - last_frame_time) < (1.0 / self.fps_limit):
                    continue
            last_frame_time = now

            # Kjør modell
            results = self.model(frame, imgsz=self.imgsz, conf=self.conf, classes=list(self.class_ids), verbose=False)

            if len(results) > 0:
                annotated_frame = results[0].plot()
            else:
                annotated_frame = frame

            # Regn ut FPS
            self.frames_processed += 1
            if (now - self.fps_calc_time) >= 1.0:
                self.current_fps = self.frames_processed / (now - self.fps_calc_time)
                self.frames_processed = 0
                self.fps_calc_time = now

            cv2.putText(annotated_frame, f"FPS: {self.current_fps:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Publiser for debug
            out = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            out.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(out)

        cap.release()

    def destroy_node(self):
        self.running = False
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        super().destroy_node()

def main():
    rclpy.init()
    node = YoloGstreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()