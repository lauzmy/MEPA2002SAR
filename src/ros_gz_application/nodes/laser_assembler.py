#!/usr/bin/env python3
"""Assemble LD06 LaserScans into a 3-D PointCloud2 for SLAM.

Pairs with ``lidar_sweeper.py`` (step-and-settle mode):

  * The sweeper holds the tilt servo *stationary* at a known angle for
    ``hold_time_s``.  ``/lidar_joint_states`` reflects that angle so
    ``robot_state_publisher`` exposes the correct laser pose via TF.
  * ``/lidar_hold_state`` is a ``std_msgs/Header`` whose ``frame_id`` is
    ``"hold"`` or ``"settle"`` and whose ``stamp`` is the start of the
    new phase.
  * ``/lidar_sweep_start`` (``std_msgs/Header``, ``frame_id="sweep_start"``)
    fires once each time the staircase wraps -- i.e. one full down-and-back
    sweep is complete.

This node:

  1. Subscribes to ``/lidar_hold_state`` and remembers the current phase
     plus its start time.
  2. For each incoming scan, accepts it iff the system is currently in
     HOLD *and* the scan started after the hold began (with a small
     ``settle_guard_s`` margin) -- i.e. the servo is provably still.
  3. Looks up the static transform ``output_frame <- scan.frame_id``
     from TF at the scan stamp and accumulates the transformed beams.
  4. On ``/lidar_sweep_start`` (or as a safety net after
     ``flush_timeout_s`` of accumulation) publishes one dense
     ``PointCloud2`` containing every accepted scan from the previous
     sweep.

Cloud fields: ``x y z intensity time ring`` -- the layout FAST-LIO /
MOLA-LO / LIO-SAM expect.

  * ``time``  : seconds since ``header.stamp`` (= sweep start), used by
                downstream nodes for motion compensation.
  * ``ring``  : monotonically increasing scan index within the sweep
                (0, 1, 2, ...), so SLAM can group beams that share a
                tilt angle.
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header

from tf2_ros import Buffer, TransformException, TransformListener


# ---------------------------------------------------------------------------
# PointCloud2 helpers (no laser_geometry / sensor_msgs_py dependency: we
# build the message directly so we control the field layout exactly the
# way generic LIO front-ends expect).
# ---------------------------------------------------------------------------
_FIELDS = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time',      offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
]
_POINT_STEP = 24  # 4*5 + 2  (the trailing 2 bytes pad the ring field)
_POINT_DTYPE = np.dtype({
    'names':    ['x', 'y', 'z', 'intensity', 'time', 'ring', '_pad'],
    'formats':  ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2', '<u2'],
    'offsets':  [0, 4, 8, 12, 16, 20, 22],
    'itemsize': _POINT_STEP,
})


def _make_pointcloud2(header: Header, rec: np.ndarray) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = rec.size
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = _POINT_STEP
    msg.row_step = _POINT_STEP * rec.size
    msg.is_dense = True
    msg.data = rec.tobytes()
    return msg


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert a unit quaternion to a 3x3 rotation matrix."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


# ---------------------------------------------------------------------------
class TiltLaserAssembler(Node):
    PHASE_HOLD = 'hold'

    def __init__(self):
        super().__init__('tilt_laser_assembler')

        # ----- Topics / frames --------------------------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/lidar3d/points')
        self.declare_parameter('hold_state_topic', '/lidar_hold_state')
        self.declare_parameter('sweep_start_topic', '/lidar_sweep_start')
        self.declare_parameter('output_frame', 'base_footprint')

        # ----- Gating / timing --------------------------------------------
        # Extra margin (on top of the sweeper's settle_time_s) before a
        # newly-begun hold window starts accepting scans.  Useful if the
        # servo has a small overshoot/oscillation tail you want to
        # discard.  0 = trust the sweeper.
        self.declare_parameter('settle_guard_s', 0.02)
        # TF lookup timeout.
        self.declare_parameter('tf_timeout_s', 0.1)
        # Safety net: if no sweep_start arrives within this many seconds
        # of the first accepted scan, flush whatever we have anyway.
        self.declare_parameter('flush_timeout_s', 15.0)
        # Don't publish near-empty clouds (e.g. on the very first sweep
        # boundary if accumulation only just started).
        self.declare_parameter('min_points_per_cloud', 200)

        scan_topic = str(self.get_parameter('scan_topic').value)
        cloud_topic = str(self.get_parameter('cloud_topic').value)
        hold_topic = str(self.get_parameter('hold_state_topic').value)
        sweep_topic = str(self.get_parameter('sweep_start_topic').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self._settle_guard = float(self.get_parameter('settle_guard_s').value)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_s').value))
        self._flush_timeout = float(self.get_parameter('flush_timeout_s').value)
        self._min_points = int(self.get_parameter('min_points_per_cloud').value)

        # ----- TF ----------------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ----- Hold-state gate --------------------------------------------
        self._in_hold = False
        self._hold_started: Time | None = None

        # ----- Sweep accumulator ------------------------------------------
        # Each element is a structured ndarray of points (one per scan).
        self._chunks: list[np.ndarray] = []
        self._sweep_start_stamp: Time | None = None
        # Last hold window we already consumed -- ensures one accepted
        # scan per hold (the first one fully inside the window).
        self._last_consumed_hold: Time | None = None
        # ring index = scans accepted in this sweep (clamped to UINT16).
        self._ring_idx = 0

        # ----- Diagnostics counters ---------------------------------------
        self._n_accepted = 0
        self._n_rejected_settle = 0
        self._n_rejected_tf = 0
        self._n_rejected_dup = 0

        # ----- ROS plumbing -----------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor_qos)
        self.create_subscription(Header, hold_topic, self._on_hold_state, 10)
        self.create_subscription(Header, sweep_topic, self._on_sweep_start, 10)
        self.pub = self.create_publisher(PointCloud2, cloud_topic, 5)

        # Periodic stats (helps tuning settle_time_s without spamming).
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f'Assembler ready: scan="{scan_topic}" -> cloud="{cloud_topic}" '
            f'in frame "{self.output_frame}", '
            f'settle_guard={self._settle_guard*1000:.0f} ms, '
            f'min_points={self._min_points}.'
        )

    # ------------------------------------------------------------------
    def _on_hold_state(self, msg: Header) -> None:
        new_in_hold = (msg.frame_id == self.PHASE_HOLD)
        if new_in_hold and not self._in_hold:
            self._hold_started = Time.from_msg(msg.stamp)
        self._in_hold = new_in_hold

    def _on_sweep_start(self, msg: Header) -> None:
        # Flush whatever we accumulated for the *previous* sweep, then
        # start a new accumulation epoch from this stamp.
        self._flush()
        self._sweep_start_stamp = Time.from_msg(msg.stamp)
        self._ring_idx = 0

    # ------------------------------------------------------------------
    def _on_scan(self, scan: LaserScan) -> None:
        # ---- gate on hold phase -----------------------------------------
        if not self._in_hold or self._hold_started is None:
            self._n_rejected_settle += 1
            return
        scan_stamp = Time.from_msg(scan.header.stamp)
        guard_ns = int(self._settle_guard * 1e9)
        if (scan_stamp - self._hold_started).nanoseconds < guard_ns:
            self._n_rejected_settle += 1
            return
        # One scan per hold window: cheap, deterministic, predictable
        # ring count per sweep.  Drop additional scans that arrive
        # before the next hold begins.
        if (self._last_consumed_hold is not None
                and self._last_consumed_hold == self._hold_started):
            self._n_rejected_dup += 1
            return

        # Need a sweep epoch before we can compute per-beam time offsets.
        if self._sweep_start_stamp is None:
            self._sweep_start_stamp = scan_stamp
            self._ring_idx = 0

        # ---- valid beams -------------------------------------------------
        n = len(scan.ranges)
        if n == 0:
            return
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = (np.isfinite(ranges)
                 & (ranges >= scan.range_min)
                 & (ranges <= scan.range_max))
        if not np.any(valid):
            return

        beam_idx = np.arange(n, dtype=np.float64)
        theta = scan.angle_min + beam_idx * scan.angle_increment

        # Per-beam time offset relative to the sweep start.
        dt = scan.time_increment if scan.time_increment > 0.0 else (1.0 / 10.0) / max(1, n)
        scan_t0_s = (scan_stamp - self._sweep_start_stamp).nanoseconds * 1e-9

        ranges_v = ranges[valid]
        theta_v = theta[valid]
        t_rel = (scan_t0_s + beam_idx[valid] * dt).astype(np.float32)

        if scan.intensities and len(scan.intensities) == n:
            intensity = np.asarray(scan.intensities, dtype=np.float32)[valid]
        else:
            intensity = np.zeros(ranges_v.size, dtype=np.float32)

        # ---- TF lookup (servo is stationary, so any time inside the
        #      hold window gives the same transform; use scan.stamp) ------
        try:
            tf = self._tf_buffer.lookup_transform(
                self.output_frame, scan.header.frame_id,
                scan_stamp, timeout=self._tf_timeout)
        except TransformException as e:
            self._n_rejected_tf += 1
            self.get_logger().warn(
                f'TF lookup {self.output_frame} <- {scan.header.frame_id} '
                f'failed: {e}', throttle_duration_sec=2.0)
            return

        # ---- transform beams into output frame ---------------------------
        x_l = ranges_v * np.cos(theta_v)
        y_l = ranges_v * np.sin(theta_v)
        pts_local = np.stack([x_l, y_l, np.zeros_like(x_l)], axis=1)

        q = tf.transform.rotation
        t = tf.transform.translation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)
        pts_out = pts_local @ R.T + translation

        # ---- accumulate --------------------------------------------------
        rec = np.empty(pts_out.shape[0], dtype=_POINT_DTYPE)
        rec['x']         = pts_out[:, 0].astype(np.float32)
        rec['y']         = pts_out[:, 1].astype(np.float32)
        rec['z']         = pts_out[:, 2].astype(np.float32)
        rec['intensity'] = intensity
        rec['time']      = t_rel
        rec['ring']      = np.uint16(min(self._ring_idx, 0xFFFF))
        rec['_pad']      = 0
        self._chunks.append(rec)
        self._ring_idx += 1
        self._last_consumed_hold = self._hold_started
        self._n_accepted += 1

        # ---- safety-net flush (in case sweep_start is lost) --------------
        if scan_t0_s >= self._flush_timeout:
            self.get_logger().warn(
                f'flush_timeout_s={self._flush_timeout:.1f}s exceeded '
                f'without /lidar_sweep_start; flushing {self._ring_idx} scans.'
            )
            self._flush()

    # ------------------------------------------------------------------
    def _flush(self) -> None:
        if not self._chunks or self._sweep_start_stamp is None:
            self._chunks.clear()
            return
        rec = np.concatenate(self._chunks)
        if rec.size < self._min_points:
            self.get_logger().debug(
                f'Skipping cloud with only {rec.size} points '
                f'(min={self._min_points}).'
            )
            self._chunks.clear()
            return

        header = Header()
        header.stamp = self._sweep_start_stamp.to_msg()
        header.frame_id = self.output_frame
        cloud = _make_pointcloud2(header, rec)
        self.pub.publish(cloud)

        self.get_logger().info(
            f'Published cloud: {rec.size} pts, {self._ring_idx} scans, '
            f'span={float(rec["time"].max()):.2f}s.'
        )
        self._chunks.clear()

    # ------------------------------------------------------------------
    def _log_stats(self) -> None:
        total = (self._n_accepted + self._n_rejected_settle
                 + self._n_rejected_tf + self._n_rejected_dup)
        if total == 0:
            return
        self.get_logger().info(
            f'5s window: accepted={self._n_accepted}  '
            f'rejected[settle]={self._n_rejected_settle}  '
            f'rejected[dup]={self._n_rejected_dup}  '
            f'rejected[tf]={self._n_rejected_tf}'
        )
        self._n_accepted = 0
        self._n_rejected_settle = 0
        self._n_rejected_tf = 0
        self._n_rejected_dup = 0


def main(args=None):
    rclpy.init(args=args)
    node = TiltLaserAssembler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
