#!/usr/bin/env python3
"""Assemble LD06 LaserScans into 3-D PointCloud2 frames for SLAM.

Pairs with ``lidar_sweeper.py`` (step-and-settle mode).

Gate
----
Scans are accepted only while the sweeper reports ``frame_id="hold"`` on
``/lidar_hold_state``.  Any other value ("settle", "calibrating") causes
the scan to be silently dropped.  An additional ``settle_guard_s`` margin
discards the first few milliseconds of each hold window in case of servo
overshoot.

Aggregation modes  (``aggregate_mode`` parameter)
--------------------------------------------------
``scan``
    Publish one PointCloud2 per accepted LaserScan.  Equivalent to a
    standard 2-D scan transformed into 3-D -- useful for debugging but
    not a real 3-D frame.

``half_sweep``
    Accumulate scans until the servo direction reverses (min→max or
    max→min), then publish one cloud per half-sweep.

``sweep``  (default)
    Accumulate across two consecutive half-sweeps (one full triangle
    cycle, min→max→min) and publish once per sweep.  Each published
    cloud typically contains ~50 rings × ~250 valid beams = ~12 500 pts.

Direction detection
-------------------
Direction is inferred from consecutive non-duplicate ``/lidar_joint_states``
position values -- no analytical wave model.  A reversal is detected when
the sign of the position delta flips.

PointCloud2 layout
------------------
Fields: x y z intensity time ring  (matches FAST-LIO / MOLA-LO / LIO-SAM).
``time`` is the per-beam offset from ``header.stamp`` (first scan in cloud)
in seconds, cast to float32.  ``ring`` is a zero-based scan index within
the cloud (wraps at 65535).
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


# ---------------------------------------------------------------------------
# PointCloud2 field layout (matches FAST-LIO / MOLA-LO / LIO-SAM)
# ---------------------------------------------------------------------------
_FIELDS = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time',      offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
]
_POINT_STEP = 24   # 5×float32 (20 B) + uint16 (2 B) + 2 B pad
_POINT_DTYPE = np.dtype({
    'names':    ['x', 'y', 'z', 'intensity', 'time', 'ring', '_pad'],
    'formats':  ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2', '<u2'],
    'offsets':  [0, 4, 8, 12, 16, 20, 22],
    'itemsize': _POINT_STEP,
})


def _make_pointcloud2(header: Header, xyz: np.ndarray, intensity: np.ndarray,
                      t_rel: np.ndarray, ring: np.ndarray) -> PointCloud2:
    n = xyz.shape[0]
    rec = np.empty(n, dtype=_POINT_DTYPE)
    rec['x']         = xyz[:, 0]
    rec['y']         = xyz[:, 1]
    rec['z']         = xyz[:, 2]
    rec['intensity'] = intensity
    rec['time']      = t_rel
    rec['ring']      = ring
    rec['_pad']      = 0
    msg = PointCloud2()
    msg.header       = header
    msg.height       = 1
    msg.width        = n
    msg.fields       = _FIELDS
    msg.is_bigendian = False
    msg.point_step   = _POINT_STEP
    msg.row_step     = _POINT_STEP * n
    msg.is_dense     = True
    msg.data         = rec.tobytes()
    return msg


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2*(yy+zz),   2*(xy-wz),   2*(xz+wy)],
        [  2*(xy+wz),   1 - 2*(xx+zz), 2*(yz-wx)],
        [  2*(xz-wy),     2*(yz+wx), 1 - 2*(xx+yy)],
    ], dtype=np.float64)


# ---------------------------------------------------------------------------
class TiltLaserAssembler(Node):

    def __init__(self):
        super().__init__('tilt_laser_assembler')

        # ----- Parameters --------------------------------------------------
        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('cloud_topic',        '/lidar3d/points')
        self.declare_parameter('hold_state_topic',   '/lidar_hold_state')
        self.declare_parameter('joint_state_topic',  '/lidar_joint_states')
        self.declare_parameter('joint_name',         'lidar_joint')
        self.declare_parameter('output_frame',       'base_footprint')
        self.declare_parameter('settle_guard_s',     0.02)
        self.declare_parameter('tf_timeout_s',       0.1)
        # 'scan' | 'half_sweep' | 'sweep'
        self.declare_parameter('aggregate_mode',     'sweep')
        # Safety net: flush if we accumulate more than 2x this many rings
        self.declare_parameter('scans_per_sweep',    50)

        scan_topic   = str(self.get_parameter('scan_topic').value)
        cloud_topic  = str(self.get_parameter('cloud_topic').value)
        hold_topic   = str(self.get_parameter('hold_state_topic').value)
        js_topic     = str(self.get_parameter('joint_state_topic').value)
        self._jname  = str(self.get_parameter('joint_name').value)
        self._frame  = str(self.get_parameter('output_frame').value)
        self._guard  = float(self.get_parameter('settle_guard_s').value)
        self._tf_tmo = Duration(seconds=float(self.get_parameter('tf_timeout_s').value))
        self._mode   = str(self.get_parameter('aggregate_mode').value)
        self._safety = int(self.get_parameter('scans_per_sweep').value) * 2

        # ----- TF ----------------------------------------------------------
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ----- Hold gate state --------------------------------------------
        self._in_hold = False
        self._hold_t0: Time | None = None

        # ----- Direction detection from joint states ----------------------
        # Track last position that was meaningfully different from its
        # predecessor to detect when the servo reverses direction.
        self._last_pos: float | None = None
        self._last_dir: int | None = None
        self._reversal_count = 0

        # ----- Cloud accumulator ------------------------------------------
        # Each chunk: (xyz f32 (N,3), intensity f32 (N,), scan_t0 float, t_within_scan f32 (N,))
        self._chunks: list[tuple[np.ndarray, np.ndarray, float, np.ndarray]] = []
        self._cloud_t0: float | None = None   # stamp of first scan in this cloud
        self._cloud_rings = 0                  # number of accepted scans so far
        self._cloud_dropped = 0                # scans rejected during accumulation

        # ----- Diagnostics ------------------------------------------------
        self._stat_published = 0
        self._stat_rej_settle = 0
        self._stat_rej_tf = 0

        # ----- ROS plumbing -----------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan,  scan_topic,  self._on_scan,       sensor_qos)
        self.create_subscription(Header,     hold_topic,  self._on_hold_state, 10)
        self.create_subscription(JointState, js_topic,    self._on_js,         10)
        self.pub = self.create_publisher(PointCloud2, cloud_topic, 5)
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f'Assembler ready  mode="{self._mode}"  '
            f'"{scan_topic}" -> "{cloud_topic}"  frame="{self._frame}"'
        )

    # ------------------------------------------------------------------
    def _on_hold_state(self, msg: Header) -> None:
        new_hold = (msg.frame_id == 'hold')
        if new_hold and not self._in_hold:
            self._hold_t0 = Time.from_msg(msg.stamp)
        self._in_hold = new_hold

    # ------------------------------------------------------------------
    def _on_js(self, msg: JointState) -> None:
        """Detect servo direction reversals from consecutive JS position deltas."""
        if self._jname not in msg.name:
            return
        pos = float(msg.position[list(msg.name).index(self._jname)])

        # Skip unchanged positions (HOLD periods publish the same value repeatedly)
        if self._last_pos is None or abs(pos - self._last_pos) < 1e-4:
            return

        new_dir = 1 if pos > self._last_pos else -1
        if self._last_dir is not None and new_dir != self._last_dir:
            self._on_reversal()
        self._last_dir = new_dir
        self._last_pos = pos

    def _on_reversal(self) -> None:
        self._reversal_count += 1
        if self._mode == 'half_sweep':
            self._publish_cloud()
        elif self._mode == 'sweep' and self._reversal_count % 2 == 0:
            self._publish_cloud()
        # In 'sweep' mode on odd reversals: keep accumulating.

    # ------------------------------------------------------------------
    def _on_scan(self, scan: LaserScan) -> None:
        # ---- gate: only accept during a hold window ---------------------
        if not self._in_hold or self._hold_t0 is None:
            self._stat_rej_settle += 1
            self._cloud_dropped += 1
            return
        scan_stamp = Time.from_msg(scan.header.stamp)
        if (scan_stamp - self._hold_t0).nanoseconds < int(self._guard * 1e9):
            self._stat_rej_settle += 1
            self._cloud_dropped += 1
            return

        # ---- valid beam mask --------------------------------------------
        n = len(scan.ranges)
        if n == 0:
            return
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = (np.isfinite(ranges)
                 & (ranges >= scan.range_min) & (ranges <= scan.range_max))
        if not np.any(valid):
            return

        beam_idx = np.arange(n, dtype=np.float64)
        theta = scan.angle_min + beam_idx * scan.angle_increment
        dt = scan.time_increment if scan.time_increment > 0.0 else (0.1 / max(1, n))
        t_in_scan = (beam_idx * dt).astype(np.float32)[valid]

        ranges = ranges[valid]
        theta = theta[valid]
        intensity = (np.asarray(scan.intensities, dtype=np.float32)[valid]
                     if scan.intensities and len(scan.intensities) == n
                     else np.zeros(ranges.size, dtype=np.float32))

        # ---- TF lookup --------------------------------------------------
        try:
            tf = self._tf_buf.lookup_transform(
                self._frame, scan.header.frame_id, scan_stamp, timeout=self._tf_tmo)
        except TransformException as e:
            self._stat_rej_tf += 1
            self.get_logger().warn(f'TF failed: {e}', throttle_duration_sec=2.0)
            return

        # ---- transform into output frame --------------------------------
        pts = np.stack([ranges * np.cos(theta), ranges * np.sin(theta),
                        np.zeros(ranges.size)], axis=1)
        q, t = tf.transform.rotation, tf.transform.translation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        xyz = (pts @ R.T + np.array([t.x, t.y, t.z])).astype(np.float32)

        scan_t0 = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9

        # ---- publish per-scan (scan mode) --------------------------------
        if self._mode == 'scan':
            ring_arr = np.full(xyz.shape[0], self._cloud_rings & 0xFFFF, dtype=np.uint16)
            h = Header()
            h.stamp = scan.header.stamp
            h.frame_id = self._frame
            self.pub.publish(_make_pointcloud2(h, xyz, intensity, t_in_scan, ring_arr))
            self._cloud_rings = (self._cloud_rings + 1) & 0xFFFF
            self._stat_published += 1
            return

        # ---- accumulate (half_sweep / sweep mode) -----------------------
        if self._cloud_t0 is None:
            self._cloud_t0 = scan_t0
            self._cloud_rings = 0
            self._cloud_dropped = 0

        self._chunks.append((xyz, intensity, scan_t0, t_in_scan))
        self._cloud_rings += 1

        # Safety net: flush if direction detection missed a reversal
        if self._cloud_rings >= self._safety:
            self.get_logger().warn(
                f'Safety flush at {self._cloud_rings} rings (no reversal detected).')
            self._publish_cloud()

    # ------------------------------------------------------------------
    def _publish_cloud(self) -> None:
        if not self._chunks:
            self._cloud_t0 = None
            self._cloud_rings = 0
            return

        cloud_t0 = self._cloud_t0
        all_xyz  = np.concatenate([c[0] for c in self._chunks], axis=0)
        all_int  = np.concatenate([c[1] for c in self._chunks], axis=0)
        # Per-beam time = offset from cloud start (first scan stamp)
        all_trel = np.concatenate(
            [(c[2] - cloud_t0 + c[3]).astype(np.float32) for c in self._chunks])
        # Ring index = scan index within the cloud (0, 1, 2, ...)
        all_ring = np.concatenate(
            [np.full(len(c[0]), i & 0xFFFF, dtype=np.uint16)
             for i, c in enumerate(self._chunks)])

        sec  = int(cloud_t0)
        nsec = int(round((cloud_t0 - sec) * 1e9))
        h = Header()
        h.stamp.sec     = sec
        h.stamp.nanosec = nsec
        h.frame_id      = self._frame

        self.pub.publish(_make_pointcloud2(h, all_xyz, all_int, all_trel, all_ring))

        n_rings = len(self._chunks)
        n_pts   = all_xyz.shape[0]
        span    = float(all_trel.max()) if all_trel.size > 0 else 0.0
        self.get_logger().info(
            f'Cloud: {n_rings} rings, {n_pts} pts, '
            f'span={span:.2f} s, dropped={self._cloud_dropped}')
        self._stat_published += 1

        self._chunks.clear()
        self._cloud_t0    = None
        self._cloud_rings = 0
        self._cloud_dropped = 0

    # ------------------------------------------------------------------
    def _log_stats(self) -> None:
        if self._stat_published + self._stat_rej_settle + self._stat_rej_tf == 0:
            return
        self.get_logger().info(
            f'5s: published={self._stat_published}  '
            f'rej[settle]={self._stat_rej_settle}  rej[tf]={self._stat_rej_tf}')
        self._stat_published = self._stat_rej_settle = self._stat_rej_tf = 0


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
