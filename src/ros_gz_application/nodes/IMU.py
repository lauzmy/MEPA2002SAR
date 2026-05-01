#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR
)

class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        
        self.get_logger().info("Initialiserer BNO085 over I2C...")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = BNO08X_I2C(i2c)
        
        # Activating the needed data from IMU
        self.sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        self.sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        self.timer = self.create_timer(0.02, self.publish_data) # 50Hz

    def publish_data(self):
        msg = Imu()
        
        # 1. Header (Time and TF Frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link' # Has to match URDF / robot state publisher

        try:
            # 2. Orientation (Quaternion)
            quat = self.sensor.quaternion
            # Did we acctual get valid data?
            if quat:
                msg.orientation.x = quat[0]
                msg.orientation.y = quat[1]
                msg.orientation.z = quat[2]
                msg.orientation.w = quat[3]

            # 3. Gyro (angular velocity in rad/s)
            gyro = self.sensor.gyro
            if gyro:
                msg.angular_velocity.x = gyro[0]
                msg.angular_velocity.y = gyro[1]
                msg.angular_velocity.z = gyro[2]

            # 4. Accelerometer (linear acceleration in m/s^2)
            accel = self.sensor.acceleration
            if accel:
                msg.linear_acceleration.x = accel[0]
                msg.linear_acceleration.y = accel[1]
                msg.linear_acceleration.z = accel[2]

            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f"Wrong reading from IMU: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()