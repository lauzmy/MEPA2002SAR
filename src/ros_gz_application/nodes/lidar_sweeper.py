#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class LidarSweeper(Node):
    # Servo calibration
    PWM_CENTER = 7.2  # Center position duty cycle
    PWM_RANGE  = 2.5  # Duty cycle range per 90 degrees

    def __init__(self):
        super().__init__('lidar_sweeper')

        self.declare_parameter('sim', True)
        self.declare_parameter('amplitude', math.radians(30))  # radians (~30 deg)
        self.declare_parameter('speed', 2.0)        # rad/s

        self.sim       = self.get_parameter('sim').value
        self.amplitude = self.get_parameter('amplitude').value
        self.speed     = self.get_parameter('speed').value

        # Publisher: picked up by ros_gz_bridge in sim, or used for logging in hw mode
        self.publisher_ = self.create_publisher(Float64, '/lidar_cmd_pos', 10)

        if not self.sim:
            from rpi_hardware_pwm import HardwarePWM
            self.pwm = HardwarePWM(pwm_channel=0, hz=50, chip=0)
            self.pwm.start(0)
            self.pwm.change_duty_cycle(self.PWM_CENTER)
            time.sleep(0.5)
            self.get_logger().info('Hardware PWM initialised (channel 0, 50 Hz)')
        else:
            self.pwm = None
            self.get_logger().info('Running in simulation mode — PWM disabled')

        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        current_time = time.time() - self.start_time
        angle_rad = self.amplitude * math.sin(self.speed * current_time)

        # Publish position for Gazebo / logging
        msg = Float64()
        msg.data = angle_rad
        self.publisher_.publish(msg)

        # Drive physical servo when not in sim mode
        if self.pwm is not None:
            angle_deg  = math.degrees(angle_rad)
            duty_cycle = self.PWM_CENTER + (angle_deg * self.PWM_RANGE / 90.0)
            self.pwm.change_duty_cycle(duty_cycle)

    def destroy_node(self):
        if self.pwm is not None:
            self.get_logger().info('Centering servo and stopping PWM')
            self.pwm.change_duty_cycle(self.PWM_CENTER)
            time.sleep(0.5)
            self.pwm.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarSweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
