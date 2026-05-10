#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import gpiod
from sensor_msgs.msg import Range


class CollisionAvoidance(Node):

    def __init__(self):
        super().__init__('collision_avoidance')

        self.chip = gpiod.Chip('gpiochip4')

        # GPIO pins
        self.front_pin = 9
        self.right_pin = 10
        self.left_pin = 11
        self.rear_pin = 22

        # Create publishers
        self.front_pub = self.create_publisher(Range, '/ir/front', 10)
        self.right_pub = self.create_publisher(Range, '/ir/right', 10)
        self.left_pub = self.create_publisher(Range, '/ir/left', 10)
        self.rear_pub = self.create_publisher(Range, '/ir/rear', 10)

        # Request GPIO lines
        self.front_line = self.setup_gpio(self.front_pin, 'front')
        self.right_line = self.setup_gpio(self.right_pin, 'right')
        self.left_line = self.setup_gpio(self.left_pin, 'left')
        self.rear_line = self.setup_gpio(self.rear_pin, 'rear')

        # Sensor config
        self.obstacle_range_front = 0.04
        self.obstacle_range_right = 0.06
        self.obstacle_range_left = 0.06
        self.obstacle_range_rear = 0.04

        self.range_min = 0.02
        self.range_max = 0.50
        
        self.timer = self.create_timer(0.1, self.read_sensors)

        self.get_logger().info('IR collision node started')

    def setup_gpio(self, pin, name):
        line = self.chip.get_line(pin)
        line.request(
            consumer=f'ir_{name}',
            type=gpiod.LINE_REQ_DIR_IN
        )
        return line

    def make_range_msg(self, frame_id, obstacle_detected, obstacle_range):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.2
        msg.min_range = self.range_min
        msg.max_range = self.range_max

        if obstacle_detected:
            msg.range = obstacle_range
        else:
            msg.range = self.range_max

        return msg

    def read_sensors(self):

        # Front
        front_detected = (self.front_line.get_value() == 0)
        front_msg = self.make_range_msg('range_front_link', front_detected, self.obstacle_range_front)
        self.front_pub.publish(front_msg)

        if front_detected:
            self.get_logger().info('Obstacle detected FRONT')

        # Right
        right_detected = (self.right_line.get_value() == 0)
        right_msg = self.make_range_msg('range_right_link', right_detected, self.obstacle_range_right)
        self.right_pub.publish(right_msg)

        if right_detected:
            self.get_logger().info('Obstacle detected RIGHT')

        # Left
        left_detected = (self.left_line.get_value() == 0)
        left_msg = self.make_range_msg('range_left_link', left_detected, self.obstacle_range_left)
        self.left_pub.publish(left_msg)

        if left_detected:
            self.get_logger().info('Obstacle detected LEFT')

        # Rear
        rear_detected = (self.rear_line.get_value() == 0)
        rear_msg = self.make_range_msg('range_rear_link', rear_detected, self.obstacle_range_rear)
        self.rear_pub.publish(rear_msg)

        if rear_detected:
            self.get_logger().info('Obstacle detected REAR')

    def destroy_node(self):
        self.front_line.release()
        self.right_line.release()
        self.left_line.release()
        self.rear_line.release()

        self.chip.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()