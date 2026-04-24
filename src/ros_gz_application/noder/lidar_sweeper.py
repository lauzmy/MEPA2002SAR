#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class LidarSweeper(Node):
    def __init__(self):
        super().__init__('lidar_sweeper')
        # Topic som ros_gz_bridge videresender til Gazebo
        self.publisher_ = self.create_publisher(Float64, '/lidar_cmd_pos', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz
        self.start_time = time.time()
        
        self.amplitude = 0.0 # Radianer (ca. 30 grader opp og ned)
        self.speed = 2.0     # Hastighet på vippingen

    def timer_callback(self):
        msg = Float64()
        # Genererer en sinusbølge for jevn vipping frem og tilbake
        current_time = time.time() - self.start_time
        msg.data = self.amplitude * math.sin(self.speed * current_time)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarSweeper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()