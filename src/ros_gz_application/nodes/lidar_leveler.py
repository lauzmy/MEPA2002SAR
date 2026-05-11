#!/usr/bin/env python3
"""Publish gravity-leveled (and zeroed) variants of /lidar3d/points.

Why
---
Our 3-D cloud comes out of the lidar3d node already projected into
``base_link``. If the robot's ``base_link`` is ever tilted in world
(uneven floor, suspension wobble, mecanum-induced roll on hard turns),
that tilt is baked into every cloud and MOLA-LO has to fight it scan-by-
scan, which it sometimes loses (the resulting simplemap looks banked).

This node side-steps that by republishing each incoming cloud through
two cheap geometric corrections, both in the original ``base_link``
frame, so MOLA can be pointed at whichever variant works best:

  /lidar3d/points_level_imu
      Use the latest IMU orientation to extract gravity-relative roll &
      pitch, then rotate every point by R = Rx(-roll) · Ry(-pitch) so
      that gravity lies along -Z. Yaw is preserved. This is the one to
      use if you trust the IMU.

  /lidar3d/points_level_zero
      Pretend the robot is always perfectly level (roll = pitch = 0).
      Since the cloud is already in ``base_link``, this is effectively a
      passthrough — but published on a dedicated topic so MOLA can be
      switched between the two by a single env-var change without
      restarting anything else. Useful baseline / sanity check.

Both topics keep the original ``frame_id`` (``base_link`` by default),
header stamp, and PointCloud2 layout. We only rewrite x, y, z fields;
intensity / time / ring (and any other extra channels) pass through
untouched.

Parameters
----------
  cloud_in_topic        (str)   /lidar3d/points
  cloud_out_imu_topic   (str)   /lidar3d/points_level_imu
  cloud_out_zero_topic  (str)   /lidar3d/points_level_zero
  imu_topic             (str)   /imu/data
  imu_max_age_s         (float) 0.25  — drop IMU correction if stale
  publish_zero          (bool)  True
  publish_imu           (bool)  True
"""
from __future__ import annotations

import math
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def _quat_to_roll_pitch(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float]:
    """Extract (roll, pitch) in radians from a quaternion (xyzw).

    Uses the standard ZYX (yaw-pitch-roll) convention. We deliberately
    discard yaw — that's MOLA's job to estimate from the lidar.
    """
    # roll (X-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (Y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)  # gimbal-lock
    else:
        pitch = math.asin(sinp)

    return roll, pitch


def _level_rotation(roll: float, pitch: float) -> np.ndarray:
    """Return a 3×3 rotation that, applied to base_link points, sends the
    gravity vector to -Z. Equivalent to Rx(-roll) · Ry(-pitch).
    """
    cr, sr = math.cos(-roll), math.sin(-roll)
    cp, sp = math.cos(-pitch), math.sin(-pitch)
    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]], dtype=np.float64)
    return Rx @ Ry


class LidarLeveler(Node):
    def __init__(self) -> None:
        super().__init__("lidar_leveler")

        self.declare_parameter("cloud_in_topic", "/lidar3d/points")
        self.declare_parameter("cloud_out_imu_topic", "/lidar3d/points_level_imu")
        self.declare_parameter("cloud_out_zero_topic", "/lidar3d/points_level_zero")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_max_age_s", 0.25)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_zero", True)

        gp = self.get_parameter
        self._cloud_in = str(gp("cloud_in_topic").value)
        self._cloud_out_imu = str(gp("cloud_out_imu_topic").value)
        self._cloud_out_zero = str(gp("cloud_out_zero_topic").value)
        self._imu_topic = str(gp("imu_topic").value)
        self._imu_max_age = float(gp("imu_max_age_s").value)
        self._publish_imu = bool(gp("publish_imu").value)
        self._publish_zero = bool(gp("publish_zero").value)

        # Latest IMU sample (locked).
        self._imu_lock = threading.Lock()
        self._imu_roll: Optional[float] = None
        self._imu_pitch: Optional[float] = None
        self._imu_stamp_ns: Optional[int] = None

        # Stats — counted from the cloud callback under the GIL, no lock.
        self._n_in = 0
        self._n_out_imu = 0
        self._n_out_zero = 0
        self._n_skipped_no_imu = 0
        self._n_skipped_stale_imu = 0

        # Reliable & SENSOR_DATA-ish QoS so we don't drop bag/replay msgs.
        cloud_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        imu_qos = QoSProfile(
            depth=20,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._sub_cloud = self.create_subscription(
            PointCloud2, self._cloud_in, self._on_cloud, cloud_qos)
        self._sub_imu = self.create_subscription(
            Imu, self._imu_topic, self._on_imu, imu_qos)

        self._pub_imu = (
            self.create_publisher(PointCloud2, self._cloud_out_imu, cloud_qos)
            if self._publish_imu else None
        )
        self._pub_zero = (
            self.create_publisher(PointCloud2, self._cloud_out_zero, cloud_qos)
            if self._publish_zero else None
        )

        # Print stats every 5 s so it's obvious if IMU samples are missing.
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f"lidar_leveler ready: in={self._cloud_in} "
            f"imu_in={self._imu_topic} "
            f"out_imu={self._cloud_out_imu if self._publish_imu else '<disabled>'} "
            f"out_zero={self._cloud_out_zero if self._publish_zero else '<disabled>'}"
        )

    # ---------------------------------------------------------------- IMU
    def _on_imu(self, msg: Imu) -> None:
        roll, pitch = _quat_to_roll_pitch(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w,
        )
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                    + msg.header.stamp.nanosec)
        with self._imu_lock:
            self._imu_roll = roll
            self._imu_pitch = pitch
            self._imu_stamp_ns = stamp_ns

    # -------------------------------------------------------------- cloud
    def _on_cloud(self, msg: PointCloud2) -> None:
        self._n_in += 1

        # Zero variant: pure passthrough, just rebrand the topic.
        if self._pub_zero is not None:
            self._pub_zero.publish(msg)
            self._n_out_zero += 1

        if self._pub_imu is None:
            return

        with self._imu_lock:
            roll = self._imu_roll
            pitch = self._imu_pitch
            stamp_ns = self._imu_stamp_ns

        if roll is None or stamp_ns is None:
            self._n_skipped_no_imu += 1
            return

        cloud_stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                          + msg.header.stamp.nanosec)
        age_s = abs(cloud_stamp_ns - stamp_ns) * 1e-9
        if age_s > self._imu_max_age:
            self._n_skipped_stale_imu += 1
            return

        R = _level_rotation(roll, pitch)
        leveled = self._rotate_cloud(msg, R)
        self._pub_imu.publish(leveled)
        self._n_out_imu += 1

    # ------------------------------------------------------ rotate cloud
    def _rotate_cloud(self, msg: PointCloud2, R: np.ndarray) -> PointCloud2:
        """Apply rotation R (3×3) to the x/y/z fields of a PointCloud2,
        leaving every other field untouched.

        Implementation note: we reinterpret the raw byte buffer as a
        numpy structured array, modify the x/y/z columns in-place on a
        copy, and rebuild the message with the same fields/layout.
        """
        n_pts = msg.width * msg.height
        if n_pts == 0:
            out = PointCloud2()
            out.header = msg.header
            out.height = msg.height
            out.width = msg.width
            out.fields = msg.fields
            out.is_bigendian = msg.is_bigendian
            out.point_step = msg.point_step
            out.row_step = msg.row_step
            out.is_dense = msg.is_dense
            out.data = msg.data
            return out

        # Build structured dtype matching the PointCloud2 fields.
        dtype = self._struct_dtype(msg)
        # Read the original bytes as a structured array (zero-copy view),
        # then take a writable copy because we'll modify x/y/z.
        raw = np.frombuffer(msg.data, dtype=dtype, count=n_pts).copy()

        xyz = np.stack([raw['x'], raw['y'], raw['z']], axis=1)  # (N,3)
        xyz_rot = xyz @ R.T                                     # apply R
        raw['x'] = xyz_rot[:, 0].astype(raw['x'].dtype)
        raw['y'] = xyz_rot[:, 1].astype(raw['y'].dtype)
        raw['z'] = xyz_rot[:, 2].astype(raw['z'].dtype)

        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = raw.tobytes()
        return out

    @staticmethod
    def _struct_dtype(msg: PointCloud2) -> np.dtype:
        """Build a numpy structured dtype that matches the PointCloud2's
        on-the-wire layout, including any padding bytes between fields
        and at the end (so total itemsize == point_step)."""
        # PointField datatype enum -> (numpy dtype, size in bytes)
        type_map = {
            1: (np.int8, 1),    # INT8
            2: (np.uint8, 1),   # UINT8
            3: (np.int16, 2),   # INT16
            4: (np.uint16, 2),  # UINT16
            5: (np.int32, 4),   # INT32
            6: (np.uint32, 4),  # UINT32
            7: (np.float32, 4), # FLOAT32
            8: (np.float64, 8), # FLOAT64
        }

        fields_sorted = sorted(msg.fields, key=lambda f: f.offset)
        names: list[str] = []
        formats: list = []
        offsets: list[int] = []
        for f in fields_sorted:
            np_type, _size = type_map[f.datatype]
            names.append(f.name)
            formats.append((np_type, f.count) if f.count > 1 else np_type)
            offsets.append(f.offset)

        return np.dtype({
            'names': names,
            'formats': formats,
            'offsets': offsets,
            'itemsize': msg.point_step,
        })

    # --------------------------------------------------------------- log
    def _log_stats(self) -> None:
        self.get_logger().info(
            f"in={self._n_in} "
            f"out_imu={self._n_out_imu} out_zero={self._n_out_zero} "
            f"skipped_no_imu={self._n_skipped_no_imu} "
            f"skipped_stale_imu={self._n_skipped_stale_imu}"
        )


def main() -> None:
    rclpy.init()
    node = LidarLeveler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
