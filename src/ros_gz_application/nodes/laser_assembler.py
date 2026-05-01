#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
from laser_geometry import LaserProjection
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.time import Time

class TiltLaserAssembler(Node):
    def __init__(self):
        super().__init__('tilt_laser_assembler')

        # --- Configuration ---
        self.fixed_frame = 'odom'        # The stationary frame in the world
        self.robot_frame = 'base_footprint'   # The frame of the robot's center
        self.sweep_duration = 2.0        # How long it takes for a full sweep (seconds)

        # TF2 Setup for getting transformation between laser and world)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Tool for convertion fra LaserScan til PointCloud2
        self.laser_proj = LaserProjection()

        # State (saving points and timing throughout sweep)
        self.accumulated_points = []
        self.sweep_start_time = self.get_clock().now()

        # Publishers and Subscribers
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/assembled_cloud', 10)
        
        self.get_logger().info("Tilt Laser Assembler startet! Samler 3D-data...")

    def scan_callback(self, scan_msg):
        # 1. Convert LaserScan to a local PointCloud2 (in the laser's own rotating frame)
        cloud_local = self.laser_proj.projectLaser(scan_msg)

        try:
            # 2. Find out where the laser was in the world (odom) at the time the scan was taken
            # We use scan_msg.header.stamp to get the exact timestamp for the scan
            trans_to_fixed = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                scan_msg.header.frame_id,
                Time().to_msg(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # 3. Move the points from the laser to the fixed world frame
            cloud_fixed = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(cloud_local, trans_to_fixed)

            # 4. Extract (X, Y, Z)-coordinates and store them in our list
            points = pc2.read_points(cloud_fixed, field_names=("x", "y", "z"), skip_nans=True)
            self.accumulated_points.extend(points)

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TF-error: Could not find transformation for scan: {e}")
            return

        # 5. Check if the sweep is finished (based on time)
        now = self.get_clock().now()
        elapsed_time = (now - self.sweep_start_time).nanoseconds / 1e9
        
        if elapsed_time >= self.sweep_duration:
            self.publish_assembled_cloud(now)
            # Reset for next sweep
            self.accumulated_points = []
            self.sweep_start_time = now

    def publish_assembled_cloud(self, timestamp):
        if not self.accumulated_points:
            return

        # Make a new header for the assembled cloud (in fixed_frame first)
        header_fixed = Header()
        header_fixed.stamp = timestamp.to_msg()
        header_fixed.frame_id = self.fixed_frame

        # Make a PointCloud2 from all the points we have collected in the odom frame
        assembled_cloud_fixed = pc2.create_cloud_xyz32(header_fixed, self.accumulated_points)

        try:
            # Find the transformation from odom to the robot's current position
            trans_to_robot = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.fixed_frame,
                Time().to_msg(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Move the entire assembled 3D cloud back to the robot's frame (base_link)
            final_cloud = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(assembled_cloud_fixed, trans_to_robot)
            
            # Publish the assembled, motion-compensated 3D cloud to MOLA!
            self.pub.publish(final_cloud)
            self.get_logger().info(f"Published 3D cloud with {len(self.accumulated_points)} points.")

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Could not transform assembled cloud to {self.robot_frame}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TiltLaserAssembler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()