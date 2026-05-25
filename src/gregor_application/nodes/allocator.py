#!/usr/bin/env python3
"""Mecanum allocator: turns cmd_vel into ESP32 PWM commands over UART and publishes wheel odometry."""

# --- Imports ---
# stdlib
import math

# third-party
import serial

# ROS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

# --- Protocol (must match ESP32 firmware) ---
SERIAL_START_BYTE = 0x55
MOTOR_CMD_BYTE = 0x01
ESP32_PACKET_SIZE = 8
CRC8_POLYNOMIAL = 0x31  # MAXIM CRC8 polynomial; must match ESP32.

STATUS_OK = 0x00
STATUS_INVALID_CMD = 0x01
STATUS_CHECKSUM_ERR = 0x02
STATUS_NAMES = {STATUS_INVALID_CMD: 'invalid command', STATUS_CHECKSUM_ERR: 'checksum error'}

# --- Timing ---
WRITE_PERIOD_S = 0.05  # 20 Hz command rate to ESP32.
READ_PERIOD_S = 0.02   # 50 Hz status poll from ESP32.

# --- Frames ---
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'

# Firmware encodes battery voltage as byte = volts * 255/16 (see ESP32_Motor_Controller.ino).
VBATT_SCALE_V_PER_BYTE = 16.0 / 255.0

# --- Odometry covariance (diagonals: x, y, z, roll, pitch, yaw / vx, vy, vz, wroll, wpitch, wyaw) ---
# See wiki: Allocator/EKF-Tuning for why vy and vyaw are high on a mecanum platform.
ODOM_POSE_COVARIANCE_DIAG = (0.01, 0.01, 0.01, 0.01, 0.01, 0.01)
ODOM_TWIST_COVARIANCE_DIAG = (0.01, 0.20, 0.01, 0.01, 0.01, 0.50)


class MecanumAllocator(Node):
    def __init__(self):
        super().__init__('mecanum_allocator')

        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('lx', 0.145 / 2)
        self.declare_parameter('ly', 0.160 / 2)
        self.declare_parameter('wheel_radius', 0.075 / 2)
        self.declare_parameter('max_rpm', 170.0)
        self.declare_parameter('max_pwm', 255)

        serial_port = str(self.get_parameter('serial_port').value)
        baud_rate = int(self.get_parameter('baud_rate').value)
        self._lx = float(self.get_parameter('lx').value)
        self._ly = float(self.get_parameter('ly').value)
        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._max_rpm = float(self.get_parameter('max_rpm').value)
        self._max_pwm = int(self.get_parameter('max_pwm').value)
        self._L = self._lx + self._ly  # wheelbase term used in mecanum IK/FK.
        wheel_circumference = 2 * math.pi * self._wheel_radius
        self._ms_to_pwm_scale = (60.0 * self._max_pwm) / (wheel_circumference * self._max_rpm)
        self._rpm_to_ms_scale = wheel_circumference / 60.0

        # --- Serial ---
        try:
            self._ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to UART on {serial_port} @ {baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            self._ser = None

        # --- State ---
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_odom_time = self.get_clock().now()

        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_wz = 0.0

        # Last commanded direction bit per motor [M1, M2, M3, M4]. ESP32 reports unsigned RPM;
        # we re-sign in _read_motor_status using this.
        self._last_dir_bits = [1, 1, 1, 1]

        # --- ROS interface ---
        self._cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self._odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self._vbatt_pub = self.create_publisher(Float32, '/battery_voltage', 10)

        # --- Timers ---
        self._write_timer = self.create_timer(WRITE_PERIOD_S, self._write_motor_command)
        self._read_timer = self.create_timer(READ_PERIOD_S, self._read_motor_status)

    # --- Callbacks ---
    def _on_cmd_vel(self, msg):
        self._target_vx = msg.linear.x
        self._target_vy = msg.linear.y
        self._target_wz = -msg.angular.z

    def _write_motor_command(self):
        if self._ser is None or not self._ser.is_open:
            return

        # Mecanum inverse kinematics: M4=FL, M3=FR, M2=RL, M1=RR.
        v_M1 = self._target_vx + self._target_vy + self._L * self._target_wz
        v_M2 = self._target_vx - self._target_vy - self._L * self._target_wz
        v_M3 = self._target_vx + self._target_vy - self._L * self._target_wz
        v_M4 = self._target_vx - self._target_vy + self._L * self._target_wz

        pwm_M1, d_M1 = self._motor_pwm_and_direction(v_M1)
        pwm_M2, d_M2 = self._motor_pwm_and_direction(v_M2)
        pwm_M3, d_M3 = self._motor_pwm_and_direction(v_M3)
        pwm_M4, d_M4 = self._motor_pwm_and_direction(v_M4)
        self._last_dir_bits = [d_M1, d_M2, d_M3, d_M4]

        dir_byte = (d_M4 << 3) | (d_M3 << 2) | (d_M2 << 1) | d_M1
        payload = [SERIAL_START_BYTE, MOTOR_CMD_BYTE, pwm_M1, pwm_M2, pwm_M3, pwm_M4, dir_byte]
        payload.append(self._crc8(payload))
        self._ser.write(bytearray(payload))

    def _read_motor_status(self):
        if self._ser is None or not self._ser.is_open:
            return

        # Drain one byte at a time looking for a start byte; only proceed when the full payload is in.
        while self._ser.in_waiting >= 1:
            if self._ser.read(1)[0] != SERIAL_START_BYTE:
                continue
            if self._ser.in_waiting < ESP32_PACKET_SIZE - 1:
                return  # tail not arrived; resync on the next tick.

            payload = self._ser.read(ESP32_PACKET_SIZE - 1)
            wire_crc = payload[-1]
            if self._crc8([SERIAL_START_BYTE, *payload[:-1]]) != wire_crc:
                self.get_logger().warning('Motor status CRC mismatch; dropping frame')
                continue

            status_byte, rpm1, rpm2, rpm3, rpm4, vbatt, _ = payload
            if status_byte != STATUS_OK:
                name = STATUS_NAMES.get(status_byte, f'unknown 0x{status_byte:02x}')
                self.get_logger().warning(f'ESP32 reports status: {name}')

            signed_rpms = [r * (2 * b - 1) for r, b in zip((rpm1, rpm2, rpm3, rpm4), self._last_dir_bits)]
            self._publish_wheel_odometry(*signed_rpms)
            self._vbatt_pub.publish(Float32(data=vbatt * VBATT_SCALE_V_PER_BYTE))

    # --- Helpers ---
    def _crc8(self, data):
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ CRC8_POLYNOMIAL
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def _motor_pwm_and_direction(self, velocity_ms):
        direction = 1 if velocity_ms >= 0 else 0
        pwm = int(abs(velocity_ms) * self._ms_to_pwm_scale)
        pwm = min(255, max(0, pwm))
        return pwm, direction

    def _rpm_to_ms(self, rpm):
        return rpm * self._rpm_to_ms_scale

    def _publish_wheel_odometry(self, rpm_M1, rpm_M2, rpm_M3, rpm_M4):
        v1 = self._rpm_to_ms(rpm_M1)
        v2 = self._rpm_to_ms(rpm_M2)
        v3 = self._rpm_to_ms(rpm_M3)
        v4 = self._rpm_to_ms(rpm_M4)

        # Forward kinematics (pseudo-inverse of IK in _write_motor_command). See wiki: Allocator/Kinematics.
        vx = (v4 + v3 + v2 + v1) / 4.0
        vy = (-v4 + v3 - v2 + v1) / 4.0
        wz = -(v4 - v3 - v2 + v1) / (4.0 * self._L)

        now = self.get_clock().now()
        dt = (now - self._last_odom_time).nanoseconds / 1e9
        self._last_odom_time = now

        cos_theta = math.cos(self._theta)
        sin_theta = math.sin(self._theta)
        self._x += (vx * cos_theta - vy * sin_theta) * dt
        self._y += (vx * sin_theta + vy * cos_theta) * dt
        self._theta += wz * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        for i, c in enumerate(ODOM_POSE_COVARIANCE_DIAG):
            odom.pose.covariance[i * 7] = c
        for i, c in enumerate(ODOM_TWIST_COVARIANCE_DIAG):
            odom.twist.covariance[i * 7] = c

        self._odom_pub.publish(odom)

    def destroy_node(self):
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumAllocator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
