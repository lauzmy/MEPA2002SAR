#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO


class CollisionAvoidance(Node):
    """Read 4 digital IR sensors directly from GPIO and publish warnings."""

    def __init__(self):
        super().__init__('collision_avoidance')

        # Sensor order: [front, right, left, rear]
        self.sensor_names = ['front', 'right', 'left', 'rear']
        self.sensor_pins = [27, 17, 22, 23]

        # Most digital IR modules output HIGH when they detect an obstacle.
        self.obstacle_level = GPIO.HIGH

        GPIO.setmode(GPIO.BCM)
        for pin in self.sensor_pins:
            GPIO.setup(pin, GPIO.IN)

        self.warning_pub = self.create_publisher(String, 'obstacle_warning', 10)
        self.timer = self.create_timer(0.1, self.read_sensors)

        self.get_logger().info('CollisionAvoidance started using GPIO only')

    def read_sensors(self):
        for name, pin in zip(self.sensor_names, self.sensor_pins):
            value = GPIO.input(pin)

            if value == self.obstacle_level:
                msg = String()
                msg.data = f'Obstacle detected: {name}'
                self.warning_pub.publish(msg)
                self.get_logger().info(msg.data)

    def destroy_node(self):
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
