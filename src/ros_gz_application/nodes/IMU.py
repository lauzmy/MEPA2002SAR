import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno08x

class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno08x.BNO08X(i2c)
        self.timer = self.create_timer(0.02, self.publish_data) # 50Hz

    def publish_data(self):
        imu_msg = Imu()
        # Fill imu_msg with self.sensor readings
        self.publisher.publish(imu_msg)