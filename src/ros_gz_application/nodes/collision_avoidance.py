#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import gpiod
from sensor_msgs.msg import Range


class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')

        self.chip = gpiod.Chip('gpiochip4')  # Raspberry Pi 5

        # Cytron IR05 GPIO pins
        self.sensors = {
            'front': 9,
            'right': 10,
            'left': 11,
            'rear': 17,
        }

        # Publishers
        self.publishers = {
            'front': self.create_publisher(Range, '/ir/front', 10),
            'right': self.create_publisher(Range, '/ir/right', 10),
            'left': self.create_publisher(Range, '/ir/left', 10),
            'rear': self.create_publisher(Range, '/ir/rear', 10),
        }

        # Request GPIO lines once
        self.lines = {}

        for name, pin in self.sensors.items():
            line = self.chip.get_line(pin)
            line.request(
                consumer=f'ir_{name}',
                type=gpiod.LINE_REQ_DIR_IN
            )
            self.lines[name] = line

        self.obstacle_range = 0.05
        self.range_min = 0.02
        self.range_max = 0.10

        self.timer = self.create_timer(0.1, self.read_sensors)  # 10 Hz

    def read_sensors(self):
        for name, line in self.lines.items():
            gpio_value = line.get_value()

            obstacle_detected = (gpio_value == 0)

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'ir_{name}_link'

            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.2
            msg.min_range = self.range_min
            msg.max_range = self.range_max

            if obstacle_detected:
                msg.range = self.obstacle_range
                self.get_logger().info(f'Obstacle detected by {name} sensor')
            else:
                msg.range = self.range_max

            self.publishers[name].publish(msg)

    def destroy_node(self):
        for line in self.lines.values():
            line.release()

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