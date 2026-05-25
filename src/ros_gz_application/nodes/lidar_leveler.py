#!/usr/bin/env python3
"""Republishes /lidar3d/points as IMU-leveled and zero-leveled variants. See wiki: LidarLeveler/Design."""

from __future__ import annotations

# --- Imports ---
# stdlib
import math
from typing import Optional

# third-party
import numpy as np
from tf_transformations import euler_from_quaternion

# ROS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def _level_rotation(roll: float, pitch: float) -> np.ndarray:
    # Sends the gravity vector to -Z. Equivalent to Rx(-roll) · Ry(-pitch).
    cr, sr = math.cos(-roll), math.sin(-roll)
    cp, sp = math.cos(-pitch), math.sin(-pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    return Rx @ Ry


class LidarLeveler(Node):
    def __init__(self) -> None:
        super().__init__("lidar_leveler")

        # --- Parameters ---
        self.declare_parameter("cloud_in_topic", "/lidar3d/points")
        self.declare_parameter("cloud_out_imu_topic", "/lidar3d/points_level_imu")
        self.declare_parameter("cloud_out_zero_topic", "/lidar3d/points_level_zero")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_max_age_s", 0.25)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_zero", True)

        self._cloud_in = str(self.get_parameter("cloud_in_topic").value)
        self._cloud_out_imu = str(self.get_parameter("cloud_out_imu_topic").value)
        self._cloud_out_zero = str(self.get_parameter("cloud_out_zero_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._imu_max_age_s = float(self.get_parameter("imu_max_age_s").value)
        self._publish_imu = bool(self.get_parameter("publish_imu").value)
        self._publish_zero = bool(self.get_parameter("publish_zero").value)

        # --- State ---
        # Latest IMU sample as (roll, pitch, stamp_ns). Tuple rebind is atomic under the GIL,
        # so reader/writer don't need a lock — reader copies the reference once and checks None.
        self._imu_state: Optional[tuple[float, float, int]] = None

        # Stats — counted from _on_cloud under the GIL, no lock needed.
        self._n_in = 0
        self._n_out_imu = 0
        self._n_out_zero = 0
        self._n_skipped_no_imu = 0
        self._n_skipped_stale_imu = 0

        # --- ROS interface ---
        # RELIABLE so bag/replay messages aren't dropped.
        cloud_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                               history=QoSHistoryPolicy.KEEP_LAST)
        imu_qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)

        self._cloud_sub = self.create_subscription(PointCloud2, self._cloud_in, self._on_cloud, cloud_qos)
        self._imu_sub = self.create_subscription(Imu, self._imu_topic, self._on_imu, imu_qos)
        self._cloud_pub_imu = (
            self.create_publisher(PointCloud2, self._cloud_out_imu, cloud_qos)
            if self._publish_imu else None
        )
        self._cloud_pub_zero = (
            self.create_publisher(PointCloud2, self._cloud_out_zero, cloud_qos)
            if self._publish_zero else None
        )

        # Periodic stats so missing IMU samples are obvious.
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f"lidar_leveler ready: in={self._cloud_in} imu_in={self._imu_topic} "
            f"out_imu={self._cloud_out_imu if self._publish_imu else '<disabled>'} "
            f"out_zero={self._cloud_out_zero if self._publish_zero else '<disabled>'}"
        )

    # --- Callbacks ---
    def _on_imu(self, msg: Imu) -> None:
        # tf_transformations expects [x, y, z, w]; yaw is discarded (MOLA estimates it from the lidar).
        roll, pitch, _ = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._imu_state = (roll, pitch, stamp_ns)

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._n_in += 1

        # Zero variant: pure passthrough on a dedicated topic.
        if self._cloud_pub_zero is not None:
            self._cloud_pub_zero.publish(msg)
            self._n_out_zero += 1

        if self._cloud_pub_imu is None or self._cloud_pub_imu.get_subscription_count() == 0:
            return

        state = self._imu_state
        if state is None:
            self._n_skipped_no_imu += 1
            return
        roll, pitch, imu_stamp_ns = state

        cloud_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if abs(cloud_stamp_ns - imu_stamp_ns) * 1e-9 > self._imu_max_age_s:
            self._n_skipped_stale_imu += 1
            return

        leveled = self._rotate_cloud(msg, _level_rotation(roll, pitch))
        self._cloud_pub_imu.publish(leveled)
        self._n_out_imu += 1

    # --- Helpers ---
    def _rotate_cloud(self, msg: PointCloud2, R: np.ndarray) -> PointCloud2:
        # Reinterpret wire bytes as a structured array, rotate x/y/z on a writable copy,
        # rebuild the message with the same layout. See wiki: LidarLeveler/PointCloud-Layout.
        n_pts = msg.width * msg.height
        if n_pts == 0:
            return self._clone_with_data(msg, msg.data)

        raw = pc2.read_points(msg).copy()
        R32 = R.astype(np.float32, copy=False)
        xyz = np.empty((n_pts, 3), dtype=np.float32)
        xyz[:, 0] = raw['x']
        xyz[:, 1] = raw['y']
        xyz[:, 2] = raw['z']
        np.matmul(xyz, R32.T, out=xyz)  # in-place; no extra allocation
        raw['x'] = xyz[:, 0]
        raw['y'] = xyz[:, 1]
        raw['z'] = xyz[:, 2]
        return self._clone_with_data(msg, raw.tobytes())

    @staticmethod
    def _clone_with_data(msg: PointCloud2, data) -> PointCloud2:
        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = data
        return out

    def _log_stats(self) -> None:
        self.get_logger().info(
            f"in={self._n_in} out_imu={self._n_out_imu} out_zero={self._n_out_zero} "
            f"skipped_no_imu={self._n_skipped_no_imu} skipped_stale_imu={self._n_skipped_stale_imu}"
        )


def main() -> None:
    rclpy.init()
    node = LidarLeveler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
