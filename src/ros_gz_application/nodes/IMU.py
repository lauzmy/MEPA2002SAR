#!/usr/bin/env python3
"""Publishes BNO085 IMU readings on /imu/data as sensor_msgs/Imu."""

from __future__ import annotations

# --- Imports ---
# stdlib
import time
from typing import Iterable

# ROS
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


def _assign_xyz(target, source) -> None:
    target.x = float(source[0])
    target.y = float(source[1])
    target.z = float(source[2])


class BNO085Node(Node):
    def __init__(self) -> None:
        super().__init__("bno085")

        # --- Parameters ---
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("report_interval_us", 40_000)
        self.declare_parameter("use_game_rotation_vector", True)
        self.declare_parameter("orientation_covariance_diagonal", [0.05, 0.05, 0.08])
        self.declare_parameter("angular_velocity_covariance_diagonal", [0.02, 0.02, 0.04])
        self.declare_parameter("linear_acceleration_covariance_diagonal", [0.2, 0.2, 0.3])

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._i2c_bus = int(self.get_parameter("i2c_bus").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        requested_interval_us = int(self.get_parameter("report_interval_us").value)
        self._use_game_rotation_vector = bool(self.get_parameter("use_game_rotation_vector").value)

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

        # --- Timing ---
        self._publish_period_s = 1.0 / publish_rate_hz
        self._report_interval_us = (
            int(round(1_000_000.0 / publish_rate_hz))
            if requested_interval_us <= 0
            else requested_interval_us
        )

        # --- Driver state ---
        self._consecutive_failures = 0
        self._quat_attr = self._load_driver()

        # --- Publishers & timers ---
        self._imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self._publish_timer = self.create_timer(self._publish_period_s, self._publish_imu_reading)

        self.get_logger().info(
            "BNO085 node started on /imu/data with frame_id=%s i2c_bus=%d report_interval_us=%d orientation=%s"
            % (
                self._frame_id,
                self._i2c_bus,
                self._report_interval_us,
                "game_rotation_vector" if self._use_game_rotation_vector else "rotation_vector",
            )
        )

    # --- Helpers ---
    def _load_driver(self) -> str:
        try:
            from adafruit_bno08x import (  # type: ignore
                BNO_REPORT_ACCELEROMETER,
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
        # BNO085 needs ~1s to boot before features can be enabled.
        time.sleep(1)

        orientation_report = (
            BNO_REPORT_GAME_ROTATION_VECTOR
            if self._use_game_rotation_vector
            else BNO_REPORT_ROTATION_VECTOR
        )

        self._bno.enable_feature(orientation_report, self._report_interval_us)
        self._bno.enable_feature(BNO_REPORT_GYROSCOPE, self._report_interval_us)
        # Raw accel (gravity included) — required by MOLA-LO. See wiki: IMU/Gravity-Correction.
        self._bno.enable_feature(BNO_REPORT_ACCELEROMETER, self._report_interval_us)
        # LINEAR_ACCELERATION enabled for diagnostics only; not published.
        self._bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION, self._report_interval_us)

        if not hasattr(self._bno, "acceleration"):
            raise RuntimeError(
                "BNO08x library does not expose 'acceleration' (raw accel "
                "with gravity). MOLA gravity correction will not work "
                "without it; upgrade adafruit-circuitpython-bno08x."
            )

        if self._use_game_rotation_vector and hasattr(self._bno, "game_quaternion"):
            return "game_quaternion"
        if self._use_game_rotation_vector:
            self.get_logger().warning(
                "BNO08x library does not expose game_quaternion, falling back to quaternion."
            )
        return "quaternion"

    def _publish_imu_reading(self) -> None:
        try:
            quaternion = getattr(self._bno, self._quat_attr)
            angular_velocity = self._bno.gyro
            linear_acceleration = self._bno.acceleration
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

        _assign_xyz(msg.angular_velocity, angular_velocity)
        msg.angular_velocity_covariance = self._angular_velocity_covariance

        _assign_xyz(msg.linear_acceleration, linear_acceleration)
        msg.linear_acceleration_covariance = self._linear_acceleration_covariance

        self._imu_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = BNO085Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
