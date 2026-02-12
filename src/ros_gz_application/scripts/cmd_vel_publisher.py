#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        self.declare_parameter('linear_x', 0.5)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('publish_hz', 10.0)

        publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value
        if publish_hz <= 0.0:
            publish_hz = 10.0
        self.timer = self.create_timer(1.0 / publish_hz, self.on_timer)

    def on_timer(self):
        msg = Twist()
        msg.linear.x = float(self.get_parameter('linear_x').value)
        msg.angular.z = float(self.get_parameter('angular_z').value)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()