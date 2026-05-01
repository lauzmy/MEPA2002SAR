#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class MecanumAllocator(Node):
    def __init__(self):
        super().__init__('mecanum_allocator')

        # Establish connection to UART
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('baud_rate', 115200)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to UART on {serial_port} with baud {baud_rate}')
        except Exception as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            self.ser = None

        # Subscribe to desired motion from e.g. teleop or navigation
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel', 
            self.cmd_vel_callback,
            10)

        # Parameters for robot's dimensions and motors
        self.declare_parameter('lx', 0.145/2)
        self.declare_parameter('ly', 0.160/2)
        self.declare_parameter('wheel_radius', 0.075/2) # Radius of the wheel in meter 
        self.declare_parameter('max_rpm', 170.0)        # Maximum RPM at 100% PWM
        self.declare_parameter('max_pwm', 255)

    def calculate_crc8(self, data):
        """Standard CRC-8-calculation for data array"""
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def cmd_vel_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            return

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        lx = self.get_parameter('lx').value
        ly = self.get_parameter('ly').value
        wheel_radius = self.get_parameter('wheel_radius').value
        max_rpm = self.get_parameter('max_rpm').value
        max_pwm = self.get_parameter('max_pwm').value

        L = lx + ly

        # Calculate kinematic velocities for each wheel in m/s
        v_M1 = vx - vy - (L * wz) # M1
        v_M2 = vx + vy + (L * wz) # M2
        v_M3 = vx + vy - (L * wz) # M3
        v_M4 = vx - vy + (L * wz) # M4

        def formater_motor_signal(hastighet_ms):
            direction = 1 if hastighet_ms >= 0 else 0
            
            # Calculate target RPM for the wheel from velocity in m/s
            rpm_target = (abs(hastighet_ms) * 60.0) / (2 * math.pi * wheel_radius)
            
            # Scale RPM requirement to a PWM signal
            pwm = int((rpm_target / max_rpm) * max_pwm)
            pwm = min(255, max(0, pwm)) # Clamp to 0-255 (1 byte)
            return pwm, direction

        pwm_M1, dir_M1 = formater_motor_signal(v_M1)
        pwm_M2, dir_M2 = formater_motor_signal(v_M2)
        pwm_M3, dir_M3 = formater_motor_signal(v_M3)
        pwm_M4, dir_M4 = formater_motor_signal(v_M4)

        # Bitmask for directions: [ ... | M4 | M3 | M2 | M1 ]
        dir_byte = (dir_M4 << 3) | (dir_M3 << 2) | (dir_M2 << 1) | dir_M1

        # Build command packet
        start_byte = 0x55
        cmd_byte = 0x01

        payload = [
            start_byte,
            cmd_byte,
            pwm_M1,
            pwm_M2,
            pwm_M3,
            pwm_M4,
            dir_byte
        ]

        # Calculate checksum (CRC-8) for payload
        crc8 = self.calculate_crc8(payload)
        payload.append(crc8)

        # Sending over UART
        self.ser.write(bytearray(payload))

def main(args=None):
    rclpy.init(args=args)
    node = MecanumAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()