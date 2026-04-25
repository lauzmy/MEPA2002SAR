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

        # --- Konfigurasjon ---
        self.fixed_frame = 'odom'        # Rammen som står stille i verden
        self.robot_frame = 'base_footprint'   # Rammen til robotens senter
        self.sweep_duration = 2.0        # Hvor lang tid ett fullt vippesveip tar (sekunder)

        # TF2 Oppsett for å hente transformasjoner (posisjoner)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Verktøy for å gjøre 2D skann om til 3D punkter
        self.laser_proj = LaserProjection()

        # State (lagring av punkter underveis i sveipet)
        self.accumulated_points = []
        self.sweep_start_time = self.get_clock().now()

        # Publishers og Subscribers
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/assembled_cloud', 10)
        
        self.get_logger().info("Tilt Laser Assembler startet! Samler 3D-data...")

    def scan_callback(self, scan_msg):
        # 1. Gjør om 2D-skann til en lokal PointCloud2 (i laserens egen roterende ramme)
        cloud_local = self.laser_proj.projectLaser(scan_msg)

        try:
            # 2. Finn ut hvor laseren var i verden (odom) akkurat da bildet ble tatt
            # Vi bruker scan_msg.header.stamp for å få nøyaktig tidspunkt for skannet
            trans_to_fixed = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                scan_msg.header.frame_id,
                Time().to_msg(),       # <--- HENT ALLTID DET NYESTE!
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # 3. Flytt punktene fra laseren og ut i den faste verdensrammen
            cloud_fixed = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(cloud_local, trans_to_fixed)

            # 4. Trekk ut (X, Y, Z)-koordinatene og lagre dem i listen vår
            points = pc2.read_points(cloud_fixed, field_names=("x", "y", "z"), skip_nans=True)
            self.accumulated_points.extend(points)

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TF-feil: Kunne ikke finne transformasjon for skann: {e}")
            return

        # 5. Sjekk om sveipet er ferdig (basert på tid)
        now = self.get_clock().now()
        elapsed_time = (now - self.sweep_start_time).nanoseconds / 1e9
        
        if elapsed_time >= self.sweep_duration:
            self.publish_assembled_cloud(now)
            # Nullstill for neste sveip
            self.accumulated_points = []
            self.sweep_start_time = now

    def publish_assembled_cloud(self, timestamp):
        if not self.accumulated_points:
            return

        # Lag en ny header for den sammensatte skyen (i fixed_frame først)
        header_fixed = Header()
        header_fixed.stamp = timestamp.to_msg()
        header_fixed.frame_id = self.fixed_frame

        # Lag en PointCloud2 fra alle punktene vi har samlet i odom-rammen
        assembled_cloud_fixed = pc2.create_cloud_xyz32(header_fixed, self.accumulated_points)

        try:
            # Finn transformasjonen fra odom og tilbake til robotens NÅVÆRENDE posisjon
            trans_to_robot = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.fixed_frame,
                Time().to_msg(),       # <--- HENT ALLTID DET NYESTE HER OGSÅ
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Flytt hele den ferdige 3D-skyen tilbake til robotens ramme (base_link)
            final_cloud = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(assembled_cloud_fixed, trans_to_robot)
            
            # Publiser den ferdige, bevegelses-kompenserte 3D-skyen til MOLA!
            self.pub.publish(final_cloud)
            self.get_logger().info(f"Publiserte 3D-sky med {len(self.accumulated_points)} punkter.")

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Kunne ikke transformere ferdig sky til {self.robot_frame}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TiltLaserAssembler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()