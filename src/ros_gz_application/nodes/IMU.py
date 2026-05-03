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
        msg.header.frame_id = 'imu_link'

        try:
            # 2. Hent all dataene
            quat = self.sensor.quaternion
            gyro = self.sensor.gyro
            accel = self.sensor.acceleration

            # 3. KUN Publiser hvis vi faktisk har gyldig data fra sensoren
            if quat and gyro and accel:
                msg.orientation.x = quat[0]
                msg.orientation.y = quat[1]
                msg.orientation.z = quat[2]
                msg.orientation.w = quat[3]

                msg.angular_velocity.x = gyro[0]
                msg.angular_velocity.y = gyro[1]
                msg.angular_velocity.z = gyro[2]

                msg.linear_acceleration.x = accel[0]
                msg.linear_acceleration.y = accel[1]
                msg.linear_acceleration.z = accel[2]

                # Sett små covariances så EKF matrisen ikke kræsjer
                msg.orientation_covariance[0] = 0.01
                msg.orientation_covariance[4] = 0.01
                msg.orientation_covariance[8] = 0.01
                
                msg.angular_velocity_covariance[0] = 0.01
                msg.angular_velocity_covariance[4] = 0.01
                msg.angular_velocity_covariance[8] = 0.01
                
                msg.linear_acceleration_covariance[0] = 0.01
                msg.linear_acceleration_covariance[4] = 0.01
                msg.linear_acceleration_covariance[8] = 0.01

                self.publisher.publish(msg)
            else:
                # Forteller oss at sensoren fremdeles varmer opp
                pass
            
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