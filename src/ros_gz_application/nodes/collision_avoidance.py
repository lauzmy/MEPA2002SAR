#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import spidev
import RPi.GPIO as GPIO
import time


class CollisionAvoidance(Node):
    """Read 4 IR sensors via SPI and publish obstacle warnings."""

    def __init__(self):
        super().__init__('collision_avoidance')

        # Sensor configuration
        self.sensor_names = ['front', 'right', 'left', 'rear']
        self.cs_pins = [27, 17, 22, 23]  # CS pins for each sensor
        self.threshold = 20  # Threshold: > 20 = obstacle, ≤ 20 = nothing

        # Setup GPIO for CS pins
        GPIO.setmode(GPIO.BCM)
        for pin in self.cs_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)

        # Setup SPI
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1000000

        # Publisher for warnings
        self.warning_pub = self.create_publisher(String, 'obstacle_warning', 10)

        # Timer to read sensors every 0.1 seconds
        self.timer = self.create_timer(0.1, self.read_sensors)

        self.get_logger().info('IR Sensor Node started')

    def read_sensor(self, cs_pin):
        """Read raw value from one IR sensor via SPI."""
        # Pull CS low
        GPIO.output(cs_pin, GPIO.LOW)
        time.sleep(0.001)

        # Read 2 bytes from SPI
        data = self.spi.readbytes(2)

        # Pull CS high
        GPIO.output(cs_pin, GPIO.HIGH)

        # Convert bytes to 16-bit value
        raw_value = (data[0] << 8) | data[1]
        return raw_value

    def read_sensors(self):
        """Read all 4 sensors and publish warnings for obstacles."""
        for name, cs_pin in zip(self.sensor_names, self.cs_pins):
            raw_value = self.read_sensor(cs_pin)

            # Check threshold: > 20 = obstacle, ≤ 20 = nothing
            if raw_value > self.threshold:
                # Publish warning
                msg = String()
                msg.data = f'Obstacle detected: {name} (value: {raw_value})'
                self.warning_pub.publish(msg)
                self.get_logger().info(msg.data)

    def destroy_node(self):
        """Cleanup GPIO and SPI."""
        self.spi.close()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
