#!/usr/bin/env python3
"""Reads 4 digital IR sensors over libgpiod and publishes sensor_msgs/Range per direction."""

import gpiod
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

SENSOR_NAMES = ('front', 'right', 'left', 'rear')
GPIO_CHIP = 'gpiochip4'  # Pi 5: RP1 bank where the 40-pin header lives.
PUBLISH_PERIOD_S = 0.1
FIELD_OF_VIEW_RAD = 0.2
RANGE_MIN_M = 0.02
RANGE_MAX_M = 0.50


class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')

        # Pin order matches SENSOR_NAMES: front, right, left, rear.
        self.declare_parameter('sensor_pins', [9, 10, 11, 22])
        # Per-sensor obstacle range (m). Front/rear set tighter than sides to match nav2 footprint.
        self.declare_parameter('obstacle_ranges_m', [0.04, 0.06, 0.06, 0.04])

        pins = list(self.get_parameter('sensor_pins').value)
        obstacle_ranges = list(self.get_parameter('obstacle_ranges_m').value)
        for name, vals in (('sensor_pins', pins), ('obstacle_ranges_m', obstacle_ranges)):
            if len(vals) != len(SENSOR_NAMES):
                raise ValueError(f'{name} must have {len(SENSOR_NAMES)} entries, got {len(vals)}')

        self._pins = dict(zip(SENSOR_NAMES, pins))
        self._obstacle_ranges = dict(zip(SENSOR_NAMES, obstacle_ranges))

        self._chip = gpiod.Chip(GPIO_CHIP)
        self._lines = {name: self._setup_line(pin, name) for name, pin in self._pins.items()}

        self._range_pubs = {
            name: self.create_publisher(Range, f'/ir/{name}', 10)
            for name in SENSOR_NAMES
        }
        self._last_obstacle_state = {name: False for name in SENSOR_NAMES}
        self._publish_timer = self.create_timer(PUBLISH_PERIOD_S, self._publish_ir_ranges)
        self.get_logger().info('IR collision node started')

    def _setup_line(self, pin: int, name: str):
        line = self._chip.get_line(pin)
        line.request(consumer=f'ir_{name}', type=gpiod.LINE_REQ_DIR_IN)
        return line

    def _publish_ir_ranges(self):
        for name in SENSOR_NAMES:
            obstacle_detected = self._lines[name].get_value() == 0

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'range_{name}_link'
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = FIELD_OF_VIEW_RAD
            msg.min_range = RANGE_MIN_M
            msg.max_range = RANGE_MAX_M
            msg.range = self._obstacle_ranges[name] if obstacle_detected else RANGE_MAX_M

            if obstacle_detected != self._last_obstacle_state[name]:
                state = 'detected' if obstacle_detected else 'cleared'
                self.get_logger().info(f'{name} sensor: obstacle {state}')
                self._last_obstacle_state[name] = obstacle_detected

            self._range_pubs[name].publish(msg)

    def destroy_node(self):
        for line in self._lines.values():
            line.release()
        self._chip.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
