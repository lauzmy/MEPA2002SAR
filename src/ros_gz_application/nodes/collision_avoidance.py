#!/usr/bin/env python3
"""Reads 4 digital IR sensors over GPIO and publishes sensor_msgs/Range per direction."""

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

SENSOR_NAMES = ('front', 'right', 'left', 'rear')
PUBLISH_PERIOD_S = 0.1
FIELD_OF_VIEW_RAD = 0.2
RANGE_MIN_M = 0.02
RANGE_MAX_M = 2.0
OBSTACLE_RANGE_M = 0.05


class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')

        # Pin order matches SENSOR_NAMES: front, right, left, rear.
        self.declare_parameter('sensor_pins', [27, 17, 22, 23])
        pins = list(self.get_parameter('sensor_pins').value)
        if len(pins) != len(SENSOR_NAMES):
            raise ValueError(f'sensor_pins must have {len(SENSOR_NAMES)} entries, got {len(pins)}')
        self._pins = dict(zip(SENSOR_NAMES, pins))

        GPIO.setmode(GPIO.BCM)
        for pin in self._pins.values():
            GPIO.setup(pin, GPIO.IN)

        self._range_pubs = {
            name: self.create_publisher(Range, f'ir/{name}', 10)
            for name in SENSOR_NAMES
        }
        self._last_obstacle_state = {name: False for name in SENSOR_NAMES}
        self._publish_timer = self.create_timer(PUBLISH_PERIOD_S, self._publish_ir_ranges)
        self.get_logger().info('CollisionAvoidance started')

    def _publish_ir_ranges(self):
        for name, pin in self._pins.items():
            obstacle_detected = GPIO.input(pin) == GPIO.LOW

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = name
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = FIELD_OF_VIEW_RAD
            msg.min_range = RANGE_MIN_M
            msg.max_range = RANGE_MAX_M
            msg.range = OBSTACLE_RANGE_M if obstacle_detected else RANGE_MAX_M

            if obstacle_detected != self._last_obstacle_state[name]:
                state = 'detected' if obstacle_detected else 'cleared'
                self.get_logger().info(f'{name} sensor: obstacle {state}')
                self._last_obstacle_state[name] = obstacle_detected

            self._range_pubs[name].publish(msg)

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
