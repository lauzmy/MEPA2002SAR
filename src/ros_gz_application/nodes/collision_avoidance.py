#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from sensor_msgs.msg import Range


class CollisionAvoidance(Node):
    """Read 4 digital IR sensors directly from GPIO and publish Range messages."""

    def __init__(self):
        super().__init__('collision_avoidance')

        # Sensor order: [front, right, left, rear]
        self.sensor_names = ['front', 'right', 'left', 'rear']
        self.sensor_pins = [27, 17, 22, 23]
        self.pub_front = self.create_publisher(Range, 'ir/front', 10)
        self.pub_right = self.create_publisher(Range, 'ir/right', 10)
        self.pub_left = self.create_publisher(Range, 'ir/left', 10)
        self.pub_rear = self.create_publisher(Range, 'ir/rear', 10)
    

        # Range uses meters, so publish a small distance when blocked and max range when clear.
        self.obstacle_range = 0.05
        self.clear_range = 2.0
        self.range_min = 0.02
        self.range_max = self.clear_range

        GPIO.setmode(GPIO.BCM)
        for pin in self.sensor_pins:
            GPIO.setup(pin, GPIO.IN)

        self.timer = self.create_timer(0.1, self.read_sensors)
        self.get_logger().info('CollisionAvoidance started using GPIO only')

    def read_sensors(self):
        for name, pin in zip(self.sensor_names, self.sensor_pins):
            obstacle_detected = GPIO.input(pin) == GPIO.LOW

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = name
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.2
            msg.min_range = self.range_min
            msg.max_range = self.range_max
            msg.range = self.obstacle_range if obstacle_detected else self.clear_range
            if name == 'front':
                self.pub_front.publish(msg)
            elif name == 'right':
                self.pub_right.publish(msg)
            elif name == 'left':
                self.pub_left.publish(msg)
            elif name == 'rear':
                self.pub_rear.publish(msg)

            self.get_logger().info('Published IR sensor readings: %s' % name)

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
