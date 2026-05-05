#!/usr/bin/env python3
"""Assemble tilted LD06 LaserScans into a 3-D PointCloud2 for MOLA-LO.

Pipeline per incoming /scan
---------------------------
1.  For every beam ``i``:
        t_i      = scan.header.stamp + i * scan.time_increment
        alpha_i  = triangle( t_i ; period, amplitude )    # deterministic
        p_laser  = ( r_i*cos(theta_i), r_i*sin(theta_i), 0 )
        # Lidar optical centre is `pivot_offset` m above the hinge along
        # the (rotating) lidar Z axis.  Tilting the hinge about Y rotates
        # both the offset and the beam.
        p_hinge  = R_y(alpha_i) @ ( p_laser + (0, 0, pivot_offset) )
        p_body   = mount_xyz + p_hinge

2.  Push (x, y, z, intensity, t_rel, ring) into an accumulator, where
    ``t_rel`` is the seconds offset from the *start* of the current
    half-sweep and ``ring`` is the index of the scan within the sweep.

3.  When the half-sweep completes (servo direction reverses) publish a
    PointCloud2 with fields  ``x y z intensity time ring``  in
    ``output_frame`` (default base_footprint).  This is the layout MOLA-LO,
    FAST-LIO, LIO-SAM etc. expect.

Why this works on the real robot
--------------------------------
There is no encoder on the tilt servo, so we cannot read the angle back.
Instead, the sweeper node and this node share an *analytical* triangle
trajectory of the form alpha(t mod period).  As long as both nodes run on
the same ROS clock and use the same parameters, the angle the assembler
computes for any beam timestamp matches the angle the servo was commanded
to within the servo's own tracking error.  The servo lag is modelled by
the ``servo_lag_s`` parameter (subtracted from each beam timestamp).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header


# ---------------------------------------------------------------------------
# Same triangle wave used by lidar_sweeper.py.  Kept inline (no import) so
# the two nodes are decoupled at the package level.
# ---------------------------------------------------------------------------
def triangle_angle(t: float, period: float, amplitude: float) -> float:
    if period <= 0.0:
        return 0.0
    phase = (t % period) / period
    if phase < 0.5:
        return amplitude * (4.0 * phase - 1.0)
    return amplitude * (3.0 - 4.0 * phase)


# Vectorised version for an array of timestamps
def triangle_angle_v(t: np.ndarray, period: float, amplitude: float) -> np.ndarray:
    if period <= 0.0:
        return np.zeros_like(t)
    phase = np.mod(t, period) / period
    rising = phase < 0.5
    out = np.empty_like(phase)
    out[rising] = amplitude * (4.0 * phase[rising] - 1.0)
    out[~rising] = amplitude * (3.0 - 4.0 * phase[~rising])
    return out


# ---------------------------------------------------------------------------
# PointCloud2 helpers (no laser_geometry / sensor_msgs_py dependency: we
# build the message directly so we control the field layout exactly the way
# MOLA-LO's generic PointCloud2 input expects).
# ---------------------------------------------------------------------------
_FIELDS = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time',      offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
]
_POINT_STEP = 24  # 4*5 + 2  (the trailing 2 bytes are part of the ring field)


def make_pointcloud2(header: Header, pts: np.ndarray) -> PointCloud2:
    """Build a PointCloud2 from an (N, 6) float-ish array.

    Columns: x, y, z, intensity, time, ring.
    """
    n = pts.shape[0]
    # Build a structured numpy array with the exact memory layout above
    dt = np.dtype({
        'names':   ['x', 'y', 'z', 'intensity', 'time', 'ring', '_pad'],
        'formats': ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2', '<u2'],
        'offsets': [0, 4, 8, 12, 16, 20, 22],
        'itemsize': _POINT_STEP,
    })
    rec = np.empty(n, dtype=dt)
    rec['x']         = pts[:, 0]
    rec['y']         = pts[:, 1]
    rec['z']         = pts[:, 2]
    rec['intensity'] = pts[:, 3]
    rec['time']      = pts[:, 4]
    rec['ring']      = pts[:, 5].astype(np.uint16)
    rec['_pad']      = 0

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = _POINT_STEP
    msg.row_step = _POINT_STEP * n
    msg.is_dense = True
    msg.data = rec.tobytes()
    return msg


# ---------------------------------------------------------------------------
class TiltLaserAssembler(Node):
    def __init__(self):
        super().__init__('tilt_laser_assembler')

        # --- Sweep geometry (must match lidar_sweeper.py) -----------------
        self.declare_parameter('amplitude_deg', 30.0)
        self.declare_parameter('lidar_scan_rate_hz', 10.0)
        self.declare_parameter('scans_per_sweep', 20)
        self.declare_parameter('servo_lag_s', 0.0)         # measured servo delay

        # --- Mechanical layout --------------------------------------------
        # Distance along the lidar Z-axis from the tilt pivot to the
        # measurement plane of the LD06.
        self.declare_parameter('pivot_offset_m', 0.0325)
        # Mount position of the tilt pivot in the output frame.
        self.declare_parameter('mount_xyz', [0.0, 0.0, 0.15])

        # --- Topics / frames ----------------------------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/lidar3d/points')
        self.declare_parameter('output_frame', 'base_footprint')
        # Some LD06 drivers (CCW) flip the angle sign vs URDF.  Set this to
        # -1.0 if the assembled cloud comes out mirrored.
        self.declare_parameter('beam_angle_sign', 1.0)

        self.amplitude = math.radians(float(self.get_parameter('amplitude_deg').value))
        scan_rate = float(self.get_parameter('lidar_scan_rate_hz').value)
        scans_per_sweep = int(self.get_parameter('scans_per_sweep').value)
        self.period = scans_per_sweep / scan_rate
        self.half_period = self.period * 0.5
        self.scans_per_half = max(1, scans_per_sweep // 2)
        self.servo_lag = float(self.get_parameter('servo_lag_s').value)

        self.pivot_offset = float(self.get_parameter('pivot_offset_m').value)
        mount = list(self.get_parameter('mount_xyz').value)
        self.mount = np.array([float(mount[0]), float(mount[1]), float(mount[2])])

        self.output_frame = str(self.get_parameter('output_frame').value)
        self.beam_sign = float(self.get_parameter('beam_angle_sign').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        cloud_topic = str(self.get_parameter('cloud_topic').value)

        # --- ROS plumbing -------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(LaserScan, scan_topic,
                                            self.scan_cb, sensor_qos)
        self.pub = self.create_publisher(PointCloud2, cloud_topic, 5)

        # --- Sweep accumulator -------------------------------------------
        self._chunks: list[np.ndarray] = []
        self._sweep_start_t: float | None = None  # ROS seconds
        self._ring_idx = 0
        # Direction we were going during the last processed scan
        # (+1 rising, -1 falling). None until first scan.
        self._last_dir: int | None = None

        self.get_logger().info(
            f'Tilt laser assembler ready '
            f'(±{math.degrees(self.amplitude):.1f}°, '
            f'period={self.period:.3f}s, '
            f'pivot offset={self.pivot_offset*1000:.1f} mm, '
            f'output frame="{self.output_frame}").'
        )

    # ------------------------------------------------------------------
    def scan_cb(self, scan: LaserScan):
        n = len(scan.ranges)
        if n == 0:
            return

        # ---- per-beam timestamps (ROS seconds, float64) ------------------
        t0 = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        # Fall back to a sensible time_increment if the driver leaves it 0.
        dt = scan.time_increment
        if dt <= 0.0:
            dt = (1.0 / 10.0) / max(1, n)  # 10 Hz scan rate
        beam_idx = np.arange(n, dtype=np.float64)
        t_beam = t0 + beam_idx * dt           # 1D, length n
        t_for_angle = t_beam - self.servo_lag  # compensate servo transport delay

        # ---- beam geometry ----------------------------------------------
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if scan.intensities and len(scan.intensities) == n:
            inten = np.asarray(scan.intensities, dtype=np.float64)
        else:
            inten = np.zeros(n, dtype=np.float64)

        theta = (scan.angle_min
                 + beam_idx * scan.angle_increment) * self.beam_sign

        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        if not np.any(valid):
            return

        ranges = ranges[valid]
        inten = inten[valid]
        theta = theta[valid]
        t_beam = t_beam[valid]
        t_for_angle = t_for_angle[valid]

        # Point in the (rotating) lidar frame: x fwd, y left, z up, z=0
        x_l = ranges * np.cos(theta)
        y_l = ranges * np.sin(theta)
        # Add the pivot-to-optical-centre offset along lidar Z
        z_l = np.full_like(x_l, self.pivot_offset)

        # ---- tilt angle per beam ----------------------------------------
        alpha = triangle_angle_v(t_for_angle, self.period, self.amplitude)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # Rotate about Y:  [ca 0 sa; 0 1 0; -sa 0 ca] @ [x;y;z]
        x_h = ca * x_l + sa * z_l
        y_h = y_l
        z_h = -sa * x_l + ca * z_l

        # Translate into the output (body) frame
        x_b = x_h + self.mount[0]
        y_b = y_h + self.mount[1]
        z_b = z_h + self.mount[2]

        # ---- accumulate --------------------------------------------------
        if self._sweep_start_t is None:
            self._sweep_start_t = float(t_beam[0])
            self._ring_idx = 0
            self._chunks.clear()

        t_rel = (t_beam - self._sweep_start_t).astype(np.float32)
        ring = np.full(t_rel.shape, self._ring_idx, dtype=np.float32)

        chunk = np.column_stack((
            x_b.astype(np.float32),
            y_b.astype(np.float32),
            z_b.astype(np.float32),
            inten.astype(np.float32),
            t_rel,
            ring,
        ))
        self._chunks.append(chunk)
        self._ring_idx += 1

        # ---- detect end of half-sweep -----------------------------------
        # We use the analytical wave to know which direction we are in.
        # When the direction flips, the half-sweep just finished -> publish.
        t_mid = float(t_beam[n // 2 if n // 2 < len(t_beam) else -1] - self.servo_lag)
        cur_dir = self._direction_at(t_mid)

        if self._last_dir is not None and cur_dir != self._last_dir:
            self._publish_and_reset(t_beam[0])
        elif self._ring_idx >= self.scans_per_half * 2:
            # Safety net: if we somehow miss a direction flip, flush after
            # a full sweep worth of scans.
            self._publish_and_reset(t_beam[0])

        self._last_dir = cur_dir

    # ------------------------------------------------------------------
    def _direction_at(self, t: float) -> int:
        phase = (t % self.period) / self.period
        return 1 if phase < 0.5 else -1

    def _publish_and_reset(self, end_stamp_s: float):
        if not self._chunks:
            self._sweep_start_t = None
            return

        pts = np.concatenate(self._chunks, axis=0)

        header = Header()
        # Stamp = start of the half-sweep (matches the convention used by
        # FAST-LIO / MOLA-LO: per-point `time` is offset from header.stamp).
        sec = int(self._sweep_start_t)
        nsec = int(round((self._sweep_start_t - sec) * 1e9))
        header.stamp.sec = sec
        header.stamp.nanosec = nsec
        header.frame_id = self.output_frame

        cloud = make_pointcloud2(header, pts)
        self.pub.publish(cloud)

        self.get_logger().info(
            f'Published 3-D cloud: {pts.shape[0]} pts, '
            f'{self._ring_idx} rings, '
            f'span={pts[:, 4].max():.3f}s.'
        )

        # reset for next half-sweep
        self._chunks.clear()
        self._sweep_start_t = None
        self._ring_idx = 0


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
