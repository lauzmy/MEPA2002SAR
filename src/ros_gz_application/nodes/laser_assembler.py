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

Angle source
------------
The assembler subscribes to the JointState topic published by lidar_sweeper
(``joint_state_topic`` parameter, default ``/lidar_joint_states``) and keeps
a rolling history of (timestamp, angle) pairs.  For each beam it interpolates
the commanded angle at that beam's timestamp using numpy.interp.  This
eliminates all phase-ambiguity from the analytical model.

The ``servo_lag_s`` parameter is still available to compensate for the servo's
mechanical tracking lag (the delay between the command and physical motion).
If the JointState topic is not available the node falls back to the analytical
triangle wave.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
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
        # When true the node estimates and corrects servo lag automatically by
        # comparing mean Z between consecutive up/down half-sweeps.
        self.declare_parameter('auto_lag', True)

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
        # JointState topic published by lidar_sweeper.  The assembler
        # interpolates the commanded angle from this history instead of
        # re-evaluating the analytical triangle wave.
        self.declare_parameter('joint_state_topic', '/lidar_joint_states')
        self.declare_parameter('joint_name', 'lidar_joint')

        self.amplitude = math.radians(float(self.get_parameter('amplitude_deg').value))
        scan_rate = float(self.get_parameter('lidar_scan_rate_hz').value)
        scans_per_sweep = int(self.get_parameter('scans_per_sweep').value)
        self.period = scans_per_sweep / scan_rate
        self.half_period = self.period * 0.5
        self.scans_per_half = max(1, scans_per_sweep // 2)
        self.servo_lag = float(self.get_parameter('servo_lag_s').value)
        self.auto_lag = bool(self.get_parameter('auto_lag').value)
        # Angular velocity of the sweep [rad/s] — constant for a triangle wave.
        self._omega = 2.0 * self.amplitude / self.half_period
        # Calibration state: last half-sweep's mean-Z and direction.
        self._cal_last_dir: int | None = None
        self._cal_last_mean_z: float | None = None
        self._cal_last_mean_r: float | None = None

        self.pivot_offset = float(self.get_parameter('pivot_offset_m').value)
        mount = list(self.get_parameter('mount_xyz').value)
        self.mount = np.array([float(mount[0]), float(mount[1]), float(mount[2])])

        self.output_frame = str(self.get_parameter('output_frame').value)
        self.beam_sign = float(self.get_parameter('beam_angle_sign').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        cloud_topic = str(self.get_parameter('cloud_topic').value)
        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self._joint_name = str(self.get_parameter('joint_name').value)

        # Rolling buffer of (ros_time_s, angle_rad) from lidar_sweeper.
        # 500 samples @ 100 Hz = 5 s of history, plenty for interpolation.
        self._angle_buf: deque[tuple[float, float]] = deque(maxlen=500)

        # --- ROS plumbing -------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(LaserScan, scan_topic,
                                            self.scan_cb, sensor_qos)
        self.pub = self.create_publisher(PointCloud2, cloud_topic, 5)
        self.js_sub = self.create_subscription(
            JointState, joint_state_topic, self._js_cb, 50)

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
    def _js_cb(self, msg: JointState):
        """Buffer stamped commanded angles from lidar_sweeper."""
        if self._joint_name not in msg.name:
            return
        idx = list(msg.name).index(self._joint_name)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._angle_buf.append((t, float(msg.position[idx])))

    def _angles_for(self, t_arr: np.ndarray) -> np.ndarray:
        """Return interpolated commanded angles for an array of timestamps.

        Falls back to the analytical triangle wave if the JointState buffer
        does not yet have enough history to cover ``t_arr``.
        """
        if len(self._angle_buf) < 2:
            # Not enough history yet — use analytical model
            return triangle_angle_v(t_arr, self.period, self.amplitude)

        buf_t = np.fromiter((p[0] for p in self._angle_buf), dtype=np.float64,
                            count=len(self._angle_buf))
        buf_a = np.fromiter((p[1] for p in self._angle_buf), dtype=np.float64,
                            count=len(self._angle_buf))
        # np.interp clamps to boundary values outside the range, which is fine.
        return np.interp(t_arr, buf_t, buf_a)

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
        alpha = self._angles_for(t_for_angle)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # Rotate about Y:  R_y(-alpha) = [ca 0 -sa; 0 1 0; sa 0 ca] @ [x;y;z]
        # Servo tilts nose-up for positive alpha, but the physical Y-axis is
        # mirrored relative to standard R_y, so we negate alpha.
        x_h = ca * x_l - sa * z_l
        y_h = y_l
        z_h = sa * x_l + ca * z_l

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

        # ---- auto lag calibration ---------------------------------------
        # The sweep has constant angular velocity omega.  A servo lag L
        # shifts all commanded angles by ±omega*L (+ on upswing, - on
        # downswing), producing a mean Z error of ±omega*L*mean(r).
        # Comparing consecutive opposite-direction sweeps:
        #   delta_z = z_up - z_down ≈ -2 * omega * L * mean_r
        #   → L_correction = -delta_z / (2 * omega * mean_r)
        # We apply a fraction of this each sweep (low-pass) for stability.
        if self.auto_lag and self._last_dir is not None:
            # cols: x y z intensity time ring
            mean_z = float(np.median(pts[:, 2]))
            # Approximate mean range from x,y,z relative to mount
            mean_r = float(np.median(np.linalg.norm(
                pts[:, :3] - self.mount, axis=1)))

            if (self._cal_last_dir is not None
                    and self._cal_last_dir != self._last_dir
                    and self._cal_last_mean_z is not None
                    and mean_r > 0.1):
                # upswing dir = +1, downswing = -1
                # On upswing, lag makes Z too low → mean_z_up < true
                # On downswing, lag makes Z too high → mean_z_down > true
                # delta = mean_z(up) - mean_z(down) = -2*omega*lag*mean_r
                if self._cal_last_dir == 1:   # last was up, current is down
                    delta_z = self._cal_last_mean_z - mean_z
                else:                          # last was down, current is up
                    delta_z = mean_z - self._cal_last_mean_z

                lag_estimate = delta_z / (2.0 * self._omega * mean_r)
                # Blend 20 % of the correction per sweep pair
                correction = 0.2 * lag_estimate
                self.servo_lag = max(-0.3, min(0.3, self.servo_lag + correction))
                self.get_logger().info(
                    f'Auto-lag: δz={delta_z*1000:.1f} mm  '
                    f'est={lag_estimate*1000:.1f} ms  '
                    f'→ servo_lag={self.servo_lag*1000:.1f} ms'
                )

            self._cal_last_dir = self._last_dir
            self._cal_last_mean_z = mean_z
            self._cal_last_mean_r = mean_r

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
