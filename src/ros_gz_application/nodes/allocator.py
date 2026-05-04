#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import math
import time

class MecanumAllocator(Node):
    def __init__(self):
        super().__init__('mecanum_allocator')

        # Establish connection to UART
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        
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
        
        self.odom_publisher = self.create_publisher(Odometry, '/gregor/odometry', 10)

        # State for odometri-kalkulering
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Variabler for å holde på den siste hastighetskommandoen
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # Timer which sends data to ESP32
        self.write_timer = self.create_timer(0.05, self.send_serial_data)

        # Timer which reads data from ESP32
        self.read_timer = self.create_timer(0.02, self.read_serial_data)

        # Parameters for robot's dimensions and motors
        self.declare_parameter('lx', 0.145/2)
        self.declare_parameter('ly', 0.160/2)
        self.declare_parameter('wheel_radius', 0.075/2) 
        self.declare_parameter('max_rpm', 170.0)        
        self.declare_parameter('max_pwm', 255)

    def calculate_crc8(self, data):
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31  # 0x31 matches MAXIM polynomial in ESP32
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def cmd_vel_callback(self, msg):
        # Instead of sending to ESP32, ONLY update the internal variables
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_wz = msg.angular.z

    def send_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            return

        lx = self.get_parameter('lx').value
        ly = self.get_parameter('ly').value
        wheel_radius = self.get_parameter('wheel_radius').value
        max_rpm = self.get_parameter('max_rpm').value
        max_pwm = self.get_parameter('max_pwm').value

        L = lx + ly

        # Calculate kinematic velocities for each wheel in m/s with saved target_vx, target_vy and target_wz
        # M4 Front-Left
        v_M4 = self.target_vx + self.target_vy + (L * self.target_wz)
        # M3 Front-Right
        v_M3 = self.target_vx - self.target_vy - (L * self.target_wz)
        # M2 Rear-Left
        v_M2 = self.target_vx + self.target_vy + (L * self.target_wz)
        # M1 Rear-Right
        v_M1 = self.target_vx - self.target_vy - (L * self.target_wz)

        def formater_motor_signal(hastighet_ms):
            direction = 1 if hastighet_ms >= 0 else 0
            rpm_target = (abs(hastighet_ms) * 60.0) / (2 * math.pi * wheel_radius)
            pwm = int((rpm_target / max_rpm) * max_pwm)
            pwm = min(255, max(0, pwm)) 
            return pwm, direction

        pwm_M1, dir_M1 = formater_motor_signal(v_M1)
        pwm_M2, dir_M2 = formater_motor_signal(v_M2)
        pwm_M3, dir_M3 = formater_motor_signal(v_M3)
        pwm_M4, dir_M4 = formater_motor_signal(v_M4)

        dir_byte = (dir_M4 << 3) | (dir_M3 << 2) | (dir_M2 << 1) | dir_M1

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

        crc8 = self.calculate_crc8(payload)
        payload.append(crc8)

        # Happens continuously X times per second via write_timer, 
        # even if cmd_vel_callback has been called or not.
        self.ser.write(bytearray(payload))


    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            return
        

        # Looking for packs which are 8 bytes
        while self.ser.in_waiting >= 8:
            # Checking that the first byte is the start byte before we read the whole block
            start_byte = self.ser.read(1)
            if start_byte[0] == 0x55:
                payload = self.ser.read(7)
                
                status_byte = payload[0]
                rpm1_byte   = payload[1]
                rpm2_byte   = payload[2]
                rpm3_byte   = payload[3]
                rpm4_byte   = payload[4]
                vbatt_byte  = payload[5]
                crc_byte    = payload[6]

                # (Valgfritt) verifiser CRC: self.calculate_crc8(bytearray([0x55]) + payload[:-1]) == crc_byte
                
                # Siden RPM og respons-retningen må hentes ut, antar vi at 'STATUS' bruker 
                # de 4 laveste bitene til retning, på samme måte som 'DIRS' da vi sendte:
                lx = self.get_parameter('lx').value
                ly = self.get_parameter('ly').value
                L = lx + ly
                
                dir_M1 = -1 if dir_M1 == 0 else 1
                dir_M2 = -1 if dir_M2 == 0 else 1
                dir_M3 = -1 if dir_M3 == 0 else 1
                dir_M4 = -1 if dir_M4 == 0 else 1

                # Converting bytes to real RPM. Sørg for at høyre side/venstre side 
                # korrigeres for speiling her hvis ESP32 ikke allerede gjør fartsvektor-speiling for RPM. 
                # Ofte er høyre motorer (M2 og M4 eller M1 og M3 avhengig av nummerering) speilet:
                rpm_M1 = rpm1_byte * dir_M1
                rpm_M2 = rpm2_byte * dir_M2
                rpm_M3 = rpm3_byte * dir_M3 
                rpm_M4 = rpm4_byte * dir_M4

                self.calculate_and_publish_odom(rpm_M1, rpm_M2, rpm_M3, rpm_M4)
            else:
                # Throwing bytes until we find the next 0x55
                pass

    def calculate_and_publish_odom(self, rpm1, rpm2, rpm3, rpm4):
        wheel_radius = self.get_parameter('wheel_radius').value
        lx = self.get_parameter('lx').value
        ly = self.get_parameter('ly').value
        L = lx + ly

        # Converting RPM per wheel back to linear wheel velocity (m/s)
        def rpm_to_ms(rpm):
            return (rpm * 2 * math.pi * wheel_radius) / 60.0

        v1 = rpm_to_ms(rpm1)
        v2 = rpm_to_ms(rpm2)
        v3 = rpm_to_ms(rpm3)
        v4 = rpm_to_ms(rpm4)

        # Forward kinematics (convert wheel velocities to robot velocities in Vx, Vy and angular velocity Wz)
        # v4=FL, v3=FR, v2=RL, v1=RR
        vx = (v4 + v3 + v2 + v1) / 4.0
        vy = (-v4 + v3 + v2 - v1) / 4.0
        wz = (-v4 + v3 - v2 + v1) / (4.0 * L)

        # 3. Odometrey integration: Calculate how much we have moved since last time
        current_time = self.get_clock().now()
        dt_duration = current_time - self.last_time
        dt = dt_duration.nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        # Rotation matrix for converting from robot frame to world frame
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # 4. Build nav_msgs/Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint" # Fra ekf_imu oppsettet ditt

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Enkel quaternion bygge from yaw (theta)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz


        # Covariance (Uncertainty). A diagonal array with 36 elements.
        # Setting a small uncertainty (e.g., 0.01) on x, y, z, roll, pitch and yaw.
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[14] = 0.01  # z
        odom.pose.covariance[21] = 0.01  # roll
        odom.pose.covariance[28] = 0.01  # pitch
        odom.pose.covariance[35] = 0.01  # yaw (theta)

        # Uncertainty for the velocity (twist)
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[7] = 0.01  # vy
        odom.twist.covariance[14] = 0.01 # vz
        odom.twist.covariance[21] = 0.01 # roll rate
        odom.twist.covariance[28] = 0.01 # pitch rate
        odom.twist.covariance[35] = 0.01 # yaw rate

        # Publish the odometry message
        self.odom_publisher.publish(odom)

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