#!/usr/bin/env python3
from __future__ import annotations

import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _diagonal_covariance(values: Iterable[float]) -> list[float]:
    diagonal = [float(value) for value in values]
    if len(diagonal) != 3:
        raise ValueError("Expected exactly 3 covariance diagonal values.")

    covariance = [0.0] * 9
    covariance[0] = diagonal[0]
    covariance[4] = diagonal[1]
    covariance[8] = diagonal[2]
    return covariance


class BNO085Node(Node):
    def __init__(self) -> None:
        super().__init__("bno085")

        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("report_interval_us", 20_000)
        self.declare_parameter("use_game_rotation_vector", True)
        self.declare_parameter("orientation_covariance_diagonal", [0.05, 0.05, 0.08])
        self.declare_parameter("angular_velocity_covariance_diagonal", [0.02, 0.02, 0.04])
        self.declare_parameter("linear_acceleration_covariance_diagonal", [0.2, 0.2, 0.3])

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._i2c_bus = self.get_parameter("i2c_bus").get_parameter_value().integer_value
        publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        requested_interval_us = self.get_parameter("report_interval_us").get_parameter_value().integer_value
        self._use_game_rotation_vector = (
            self.get_parameter("use_game_rotation_vector").get_parameter_value().bool_value
        )

        self._orientation_covariance = _diagonal_covariance(
            self.get_parameter("orientation_covariance_diagonal").value
        )
        self._angular_velocity_covariance = _diagonal_covariance(
            self.get_parameter("angular_velocity_covariance_diagonal").value
        )
        self._linear_acceleration_covariance = _diagonal_covariance(
            self.get_parameter("linear_acceleration_covariance_diagonal").value
        )

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero.")

        self._publish_period_s = 1.0 / publish_rate_hz
        self._report_interval_us = (
            int(round(1_000_000.0 / publish_rate_hz))
            if requested_interval_us <= 0
            else int(requested_interval_us)
        )
        self._consecutive_failures = 0

        self._bno = None
        self._gyro_attr = "gyro"
        self._linear_accel_attr = "linear_acceleration"
        self._quat_attr = "game_quaternion" if self._use_game_rotation_vector else "quaternion"

        self._load_driver()
        self._pub = self.create_publisher(Imu, "imu/data", 10)
        self._timer = self.create_timer(self._publish_period_s, self._on_timer)

        self.get_logger().info(
            "BNO085 node started on /imu/data with frame_id=%s i2c_bus=%d report_interval_us=%d orientation=%s"
            % (
                self._frame_id,
                self._i2c_bus,
                self._report_interval_us,
                "game_rotation_vector" if self._use_game_rotation_vector else "rotation_vector",
            )
        )

    def _load_driver(self) -> None:
        try:
            from adafruit_bno08x import (  # type: ignore
                BNO_REPORT_GAME_ROTATION_VECTOR,
                BNO_REPORT_GYROSCOPE,
                BNO_REPORT_LINEAR_ACCELERATION,
                BNO_REPORT_ROTATION_VECTOR,
            )
            from adafruit_extended_bus import ExtendedI2C  # type: ignore
            from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "Missing BNO085 dependencies. Install adafruit-circuitpython-bno08x "
                "and adafruit-extended-bus in the runtime environment."
            ) from exc

        i2c = ExtendedI2C(self._i2c_bus)
        self._bno = BNO08X_I2C(i2c)
        time.sleep(1)  # Give the sensor time to boot up
        self._bno.soft_reset()
        time.sleep(1)  # Wait for reset to complete

        orientation_report = (
            BNO_REPORT_GAME_ROTATION_VECTOR
            if self._use_game_rotation_vector
            else BNO_REPORT_ROTATION_VECTOR
        )

        self._bno.enable_feature(orientation_report, self._report_interval_us)
        self._bno.enable_feature(BNO_REPORT_GYROSCOPE, self._report_interval_us)
        self._bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION, self._report_interval_us)

        if self._use_game_rotation_vector and not hasattr(self._bno, "game_quaternion"):
            self._quat_attr = "quaternion"
            self.get_logger().warning(
                "BNO08x library does not expose game_quaternion, falling back to quaternion."
            )

        if not hasattr(self._bno, self._linear_accel_attr):
            self._linear_accel_attr = "acceleration"
            self.get_logger().warning(
                "BNO08x library does not expose linear_acceleration, falling back to acceleration."
            )

    def _on_timer(self) -> None:
        try:
            quaternion = getattr(self._bno, self._quat_attr)
            angular_velocity = getattr(self._bno, self._gyro_attr)
            linear_acceleration = getattr(self._bno, self._linear_accel_attr)
        except Exception as exc:  # pragma: no cover - hardware/runtime dependent
            self._consecutive_failures += 1
            if self._consecutive_failures in (1, 10) or self._consecutive_failures % 50 == 0:
                self.get_logger().warning(f"Failed to read BNO085 data: {exc}")
            return

        self._consecutive_failures = 0
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.orientation.x = float(quaternion[0])
        msg.orientation.y = float(quaternion[1])
        msg.orientation.z = float(quaternion[2])
        msg.orientation.w = float(quaternion[3])
        msg.orientation_covariance = self._orientation_covariance

        msg.angular_velocity.x = float(angular_velocity[0])
        msg.angular_velocity.y = float(angular_velocity[1])
        msg.angular_velocity.z = float(angular_velocity[2])
        msg.angular_velocity_covariance = self._angular_velocity_covariance

        msg.linear_acceleration.x = float(linear_acceleration[0])
        msg.linear_acceleration.y = float(linear_acceleration[1])
        msg.linear_acceleration.z = float(linear_acceleration[2])
        msg.linear_acceleration_covariance = self._linear_acceleration_covariance

        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = BNO085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()