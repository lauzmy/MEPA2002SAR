#!/usr/bin/env python3
"""3D LiDAR node — tilt sweep + sweep-accumulated 3-D cloud assembly.

Responsibilities
----------------
1. Drive the tilt servo with a continuous triangle sweep
   (`/lidar_cmd_pos` → joint controller in sim, hardware PWM on real).
2. (Real robot only) Read the *measured* tilt angle from an ESP32
   over UART and publish it as ``/lidar_joint_states`` so
   ``joint_state_publisher`` → ``robot_state_publisher`` puts the
   correct laser TF on the wire.
3. Convert every incoming ``/scan`` (LaserScan) into points in
   ``base_link`` coordinates with the per-beam tilt baked in, and
   accumulate them across one full tilt sweep before publishing a
   single merged ``PointCloud2`` on ``/lidar3d/points``.  This gives
   downstream odometry (MOLA-LO, FAST-LIO etc.) a proper 3-D scene
   instead of a single nearly-planar ring.

Frame & projection
------------------
Each beam is projected from the laser frame through the static URDF
chain ``laser → lidar_mount_link → upper_level_link → base_link``,
applying the live tilt angle around the joint axis.  Because the
URDF is built so that at tilt=0 ``lidar_mount_link`` is aligned with
``base_link`` and the joint axis equals base_link X, the math
reduces to::

    q       = R_z(π) · p_laser + (0, 0, sensor_z_offset)     # mount frame
    p_base  = R_x(tilt) · q + hinge_xyz                      # base frame

Output frame is configurable but defaults to ``base_link``.

UART packet (little-endian, 7 bytes)
------------------------------------
    [0]   sync     0xAA
    [1]   seq      uint8
    [2..3] angle   int16, centi-degrees
    [4..5] wiper   uint16, millivolts (ignored here)
    [6]   crc8     poly 0x07, init 0x00, over bytes 1..5
"""

from __future__ import annotations

import math
import threading
from collections import deque
from typing import Deque, Tuple

import numpy as np
import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float64


# ─── PointCloud2 layout (x y z intensity time ring) ───────────────────────
# `ring` is constant 0 (a single tilted ring) but FAST-LIO / LIO-SAM /
# MOLA-LO all expect the field to exist.  Two bytes/point of padding.
_FIELDS = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time',      offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
]
_POINT_STEP = 24  # 5×float32 + uint16 + 2-byte pad
_POINT_DTYPE = np.dtype({
    'names':    ['x', 'y', 'z', 'intensity', 'time', 'ring', '_pad'],
    'formats':  ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2', '<u2'],
    'offsets':  [0, 4, 8, 12, 16, 20, 22],
    'itemsize': _POINT_STEP,
})


# ─── Helpers ──────────────────────────────────────────────────────────────
def _crc8(buf: bytes) -> int:
    """CRC-8 (poly 0x07, init 0x00) — matches the ESP32 firmware."""
    c = 0
    for b in buf:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def _triangle(t: float, period: float, lo: float, hi: float) -> float:
    """Continuous triangle wave between ``lo`` and ``hi``."""
    if period <= 0.0:
        return 0.5 * (lo + hi)
    phase = (t % period) / period           # 0 .. 1
    tri = 1.0 - abs(2.0 * phase - 1.0)      # 0 .. 1 .. 0
    return lo + tri * (hi - lo)


# ─── Angle history (thread-safe ring buffer of (t, angle)) ────────────────
class AngleHistory:
    """Lock-protected buffer for interpolating the measured tilt angle."""

    def __init__(self, capacity: int = 4096):
        self._buf: Deque[Tuple[float, float]] = deque(maxlen=capacity)
        self._lock = threading.Lock()

    def push(self, t: float, angle_rad: float) -> None:
        with self._lock:
            self._buf.append((t, angle_rad))

    def latest(self) -> Tuple[float, float] | None:
        with self._lock:
            return self._buf[-1] if self._buf else None

    def snapshot(self) -> Tuple[np.ndarray, np.ndarray] | None:
        """Copy the buffer out as two parallel numpy arrays (t, angle)."""
        with self._lock:
            if not self._buf:
                return None
            arr = np.fromiter(
                (v for pair in self._buf for v in pair),
                dtype=np.float64,
                count=2 * len(self._buf),
            ).reshape(-1, 2)
        return arr[:, 0], arr[:, 1]


# ─── UART reader thread ───────────────────────────────────────────────────
class UartAngleReader(threading.Thread):
    """Reads framed packets, pushes (t, angle) samples into history."""

    SYNC = 0xAA
    PACKET_LEN = 7

    def __init__(self, port: str, baud: int, history: AngleHistory,
                 angle_offset_rad: float, invert: bool, clock, logger):
        super().__init__(daemon=True)
        self._port_name = port
        self._baud = baud
        self._history = history
        self._offset = angle_offset_rad
        self._sign = -1.0 if invert else 1.0
        self._clock = clock
        self._log = logger
        self._stop = threading.Event()
        self._ser: serial.Serial | None = None

    def stop(self) -> None:
        self._stop.set()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass

    def run(self) -> None:
        try:
            self._ser = serial.Serial(self._port_name, self._baud, timeout=0.1)
        except Exception as e:  # noqa: BLE001
            self._log.error(f'UART open failed ({self._port_name}): {e}')
            return
        self._log.info(f'UART open: {self._port_name} @ {self._baud}')

        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(64)
            except Exception as e:  # noqa: BLE001
                self._log.warn(f'UART read error: {e}')
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            self._consume(buf)

    def _consume(self, buf: bytearray) -> None:
        while len(buf) >= self.PACKET_LEN:
            if buf[0] != self.SYNC:
                del buf[0]
                continue
            packet = bytes(buf[:self.PACKET_LEN])
            if _crc8(packet[1:6]) != packet[6]:
                del buf[0]
                continue
            angle_cdeg = int.from_bytes(packet[2:4], 'little', signed=True)
            angle_rad = self._sign * math.radians(angle_cdeg / 100.0) + self._offset
            t_now = self._clock.now().nanoseconds * 1e-9
            self._history.push(t_now, angle_rad)
            del buf[:self.PACKET_LEN]


# ─── The node ─────────────────────────────────────────────────────────────
class Lidar3D(Node):
    # Servo PWM calibration (50 Hz signal; measured on the unit).
    PWM_CENTER_PCT = 7.2
    PWM_PCT_PER_DEG = 5.0 / 90.0

    def __init__(self):
        super().__init__('lidar3d')
        self._declare_params()
        self._read_params()

        self._t0 = self._now_s()
        self._cmd_angle_rad = 0.0
        self._angles = AngleHistory()
        self._n_clouds_pub = 0
        self._n_revs_in_sweep = 0

        # Sweep accumulator: per-revolution arrays in `output_frame` coords
        # with the per-beam tilt baked in. Old revolutions are evicted on
        # flush so each published cloud spans only the last
        # `sweep_period_s` seconds — this is what gives MOLA-LO real 3-D
        # structure to register against (a single tilted ring is planar
        # and ICP cannot constrain out-of-plane DoFs from one).
        self._buf: Deque[Tuple[np.ndarray, np.ndarray, np.ndarray,
                                np.ndarray, np.ndarray]] = deque()
        # Touched by both `_on_scan` (subscription thread) and (formerly) a
        # timer; future-proof against MultiThreadedExecutor by guarding it.
        self._buf_lock = threading.Lock()
        # Strict-monotonic guard on published cloud header stamps. MOLA-LO's
        # input stage rejects (and warns about) clouds whose stamp does not
        # advance, so we drop any flush that would re-use the previous one.
        self._last_published_stamp = 0.0
        # Wall-clock-style throttle for `_on_scan`-driven flushes. Uses the
        # same time source as the beam timestamps so it works under both
        # sim time and live wall time.
        self._last_flush_t = 0.0
        # Last accepted scan stamp. Used by the monotonic-scan guard in
        # `_on_scan` to detect sim-time jumps backwards (Gazebo bumps
        # /clock during world load, then resumes from a smaller value
        # once assets finish loading — a bad scan from the pre-load
        # window otherwise gets stuck as MOLA's t[k-1] reference and
        # produces an infinite "timestamps went backwards" warning).
        self._last_scan_t = 0.0

        self._init_pwm()
        self._init_uart()
        self._init_io()

        self.get_logger().info(
            f'lidar3d ready: sweep {math.degrees(self._lo):+.1f}…'
            f'{math.degrees(self._hi):+.1f} deg every {self._sweep_period:.2f} s, '
            f'cloud="{self._cloud_topic}" (frame={self._out_frame}, '
            f'window={self._sweep_period:.2f}s, publish every {self._publish_period:.2f}s).')

    # ── Parameter setup ───────────────────────────────────────────────────
    def _declare_params(self) -> None:
        # Mode
        self.declare_parameter('sim', False)

        # Tilt sweep
        self.declare_parameter('min_angle_deg', -30.0)
        self.declare_parameter('max_angle_deg', 30.0)
        self.declare_parameter('sweep_period_s', 4.0)
        # How often to flush a merged cloud. Each cloud contains beams
        # from the last `sweep_period_s` seconds, so consecutive clouds
        # overlap when publish_period_s < sweep_period_s. Set them equal
        # to publish disjoint sweeps.
        self.declare_parameter('publish_period_s', 0.5)
        self.declare_parameter('pwm_update_hz', 50.0)
        self.declare_parameter('pwm_chip', 1)     # Pi 5: pwmchip0 is RP1 internal
        self.declare_parameter('pwm_channel', 0)  # GPIO12=ch0, GPIO13=ch1

        # UART angle feedback (real robot only)
        self.declare_parameter('uart_port', '/dev/ttyAMA1')
        self.declare_parameter('uart_baud', 921600)
        self.declare_parameter('angle_offset_deg', 163.52)  # pot midpoint
        self.declare_parameter('angle_invert', False)

        # Topics / frames
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/lidar3d/points')
        self.declare_parameter('joint_state_topic', '/lidar_joint_states')
        # Sim: subscribe here to get the *measured* tilt angle out of
        # Gazebo's joint_state_publisher (bridged from
        # /world/<world>/model/<robot>/joint_state). Replaces using the
        # commanded triangle (which lags the JointPositionController).
        self.declare_parameter('joint_states_in_topic', '/joint_states')
        self.declare_parameter('cmd_topic', '/lidar_cmd_pos')
        self.declare_parameter('joint_name', 'lidar_joint')
        self.declare_parameter('joint_state_rate_hz', 50.0)

        # Geometry (base_link → tilt-axis hinge point, in metres).
        # Derived from the URDF: base_link → upper_level_link translation
        # (small XY shift) plus upper_level_link → joint origin Z.
        self.declare_parameter('hinge_xyz_m', [0.00015423, -0.007561, 0.09175145])
        # lidar_mount_link → laser link Z offset (along base_link Z at tilt=0).
        self.declare_parameter('sensor_z_offset_m', 0.00615)
        # Output frame — must be a tilt-stable parent (default: base_link)
        # because we bake the per-beam tilt into the points ourselves.
        self.declare_parameter('output_frame', 'base_link')
        # Beams shorter than this (after the LD06's own range_min) are dropped.
        # Useful to suppress points that hit the chassis on extreme tilts.
        self.declare_parameter('min_range_m', 0.10)
        # Assumed revolution period when the LaserScan driver reports
        # `time_increment == 0` (i.e. it doesn't tell us the per-beam
        # spacing). Used by `_on_scan` to spread beams uniformly across
        # the revolution so the per-beam tilt interpolation is correct.
        # Default 0.1 s = LD06 nominal 10 Hz. Set to 0.0 to disable the
        # spread (all beams stamped at scan.header.stamp, the old
        # behaviour) — only do that if you know your driver already
        # provides correct per-beam timing.
        self.declare_parameter('assumed_scan_period_s', 0.1)
        # Drop projected points whose Z (in `output_frame`, i.e. base_link)
        # falls outside [min_z_m, max_z_m]. Useful to suppress sub-floor
        # artefacts caused by tilt-angle latency between /joint_states
        # (50 Hz) and /scan beams. DISABLED by default (very large
        # bounds) because the correct threshold depends on the
        # base_footprint -> base_link Z offset of your URDF and you don't
        # want to silently cull legitimate floor returns. Tune in YAML,
        # e.g. min_z_m: -0.02 once you've measured the floor's actual Z
        # in base_link from RViz.
        self.declare_parameter('min_z_m', -1000.0)
        self.declare_parameter('max_z_m',  1000.0)

    def _read_params(self) -> None:
        gp = self.get_parameter
        self._sim = bool(gp('sim').value)

        self._lo = math.radians(float(gp('min_angle_deg').value))
        self._hi = math.radians(float(gp('max_angle_deg').value))
        self._sweep_period = float(gp('sweep_period_s').value)
        self._publish_period = float(gp('publish_period_s').value)
        self._pwm_hz = float(gp('pwm_update_hz').value)
        self._pwm_chip = int(gp('pwm_chip').value)
        self._pwm_channel = int(gp('pwm_channel').value)

        self._uart_port = str(gp('uart_port').value)
        self._uart_baud = int(gp('uart_baud').value)
        self._angle_offset = math.radians(float(gp('angle_offset_deg').value))
        self._angle_invert = bool(gp('angle_invert').value)

        self._scan_topic = str(gp('scan_topic').value)
        self._cloud_topic = str(gp('cloud_topic').value)
        self._js_topic = str(gp('joint_state_topic').value)
        self._js_in_topic = str(gp('joint_states_in_topic').value)
        self._cmd_topic = str(gp('cmd_topic').value)
        self._joint_name = str(gp('joint_name').value)
        self._js_hz = float(gp('joint_state_rate_hz').value)

        hinge = list(gp('hinge_xyz_m').value)
        self._hinge = np.asarray(hinge, dtype=np.float64).reshape(3)
        self._z_off = float(gp('sensor_z_offset_m').value)
        self._out_frame = str(gp('output_frame').value)
        self._min_range = float(gp('min_range_m').value)
        self._min_z = float(gp('min_z_m').value)
        self._max_z = float(gp('max_z_m').value)
        self._assumed_scan_period = float(gp('assumed_scan_period_s').value)

    # ── Hardware init ─────────────────────────────────────────────────────
    def _init_pwm(self) -> None:
        self._pwm = None
        if self._sim:
            self.get_logger().info('sim=True → hardware PWM disabled.')
            return
        try:
            # Unexport first so a previous crash doesn't leave the channel
            # locked (kernel returns EBUSY on re-export otherwise).
            unexport = f'/sys/class/pwm/pwmchip{self._pwm_chip}/unexport'
            try:
                with open(unexport, 'w') as f:
                    f.write(str(self._pwm_channel))
            except OSError:
                pass

            from rpi_hardware_pwm import HardwarePWM
            self._pwm = HardwarePWM(
                pwm_channel=self._pwm_channel, hz=50, chip=self._pwm_chip)
            self._pwm.start(0)
            self._pwm.change_duty_cycle(self.PWM_CENTER_PCT)
            self.get_logger().info(
                f'Hardware PWM ready (chip={self._pwm_chip}, '
                f'chan={self._pwm_channel}, 50 Hz).')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f'Hardware PWM unavailable ({e}); command-only mode.')
            self._pwm = None

    def _init_uart(self) -> None:
        self._uart = UartAngleReader(
            port=self._uart_port, baud=self._uart_baud, history=self._angles,
            angle_offset_rad=self._angle_offset, invert=self._angle_invert,
            clock=self.get_clock(), logger=self.get_logger())
        if self._sim:
            self.get_logger().info('sim=True → UART angle reader disabled.')
        else:
            self._uart.start()

    def _init_io(self) -> None:
        self._cmd_pub = self.create_publisher(Float64, self._cmd_topic, 10)
        self._js_pub = self.create_publisher(JointState, self._js_topic, 10)
        self._cloud_pub = self.create_publisher(PointCloud2, self._cloud_topic, 5)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, sensor_qos)

        # In sim, the UART reader is disabled; pull the *measured* tilt out
        # of Gazebo's /joint_states bridge instead so health logging and
        # the (unused-in-sim) /lidar_joint_states stay accurate.
        if self._sim:
            self.create_subscription(
                JointState, self._js_in_topic, self._on_joint_state, 50)

        self.create_timer(1.0 / self._pwm_hz, self._tick_sweep)
        self.create_timer(1.0 / self._js_hz, self._tick_joint_state)
        # NOTE: no separate flush timer. `_flush_sweep` is invoked from
        # `_on_scan` once per `publish_period_s` of accumulated beams. This
        # eliminates the failure mode where the timer fires with no new
        # /scan in between, producing two clouds with the same `latest_t`
        # header stamp (MOLA then logs "timestamps went backwards").
        self.create_timer(5.0, self._log_health)

    # ── Helpers ───────────────────────────────────────────────────────────
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _current_tilt_rad(self) -> float:
        """Best estimate of the current tilt angle in radians."""
        latest = self._angles.latest()
        if latest is not None:
            return latest[1]
        return self._cmd_angle_rad

    # ── Periodic tasks ────────────────────────────────────────────────────
    def _tick_sweep(self) -> None:
        angle = _triangle(self._now_s() - self._t0,
                          self._sweep_period, self._lo, self._hi)
        self._cmd_angle_rad = angle
        self._cmd_pub.publish(Float64(data=angle))
        if self._pwm is not None:
            duty = self.PWM_CENTER_PCT + math.degrees(angle) * self.PWM_PCT_PER_DEG
            self._pwm.change_duty_cycle(duty)

    def _tick_joint_state(self) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._joint_name]
        js.position = [self._current_tilt_rad()]
        js.velocity = [0.0]
        self._js_pub.publish(js)

    def _log_health(self) -> None:
        latest = self._angles.latest()
        cmd_deg = math.degrees(self._cmd_angle_rad)
        if latest is not None:
            age_ms = (self._now_s() - latest[0]) * 1e3
            angle_str = f'angle={math.degrees(latest[1]):+.2f}° ({age_ms:.0f} ms old)'
        elif self._sim:
            angle_str = 'angle=<sim, using cmd>'
        else:
            angle_str = 'angle=<no UART samples yet>'
        self.get_logger().info(
            f'5s window: clouds={self._n_clouds_pub}  '
            f'{angle_str}  cmd={cmd_deg:+.2f}°')
        self._n_clouds_pub = 0

    # ── Sim: /joint_states → measured tilt ────────────────────────────────
    def _on_joint_state(self, msg: JointState) -> None:
        """Push the measured tilt joint position into `AngleHistory` (sim)."""
        try:
            i = msg.name.index(self._joint_name)
        except ValueError:
            return
        if i >= len(msg.position):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:  # missing stamp — fall back to wall clock
            t = self._now_s()
        self._angles.push(t, float(msg.position[i]))

    # ── /scan → sweep accumulator ─────────────────────────────────────────
    def _on_scan(self, scan: LaserScan) -> None:
        """Project one LaserScan revolution and append it to the sweep buffer.

        Each beam's tilt angle is interpolated from the live tilt history
        (UART on real, /joint_states on sim) and the beam is transformed
        into the tilt-stable `output_frame` (default `base_link`). Tilt
        motion is therefore baked in here, *not* deferred to MOLA's TF
        deskewer — the cloud's per-point `time` is set to 0 so the SLAM
        front-end treats it as already undistorted.

        Nothing is published from this callback; the buffered revolutions
        are merged into a single 3-D cloud by `_flush_sweep` on a timer.
        """
        n = len(scan.ranges)
        if n == 0:
            return

        # Reject scans whose stamp is way off the node's clock. In sim
        # the bridge can deliver one or two early /scan messages stamped
        # with wall clock (or with a stale stamp from a previous Gazebo
        # session left in the bridge queue) before /clock starts
        # ticking; if buffered, they poison the sweep accumulator (their
        # off-clock stamp becomes "latest" forever and every published
        # cloud inherits it → MOLA logs "LiDAR timestamps went
        # backwards in time: t[k-1]=<old>, t[k]=<real>" indefinitely
        # and the map breaks as soon as it happens).
        #
        # In sim mode we additionally require sim clock to be valid
        # (>0) before accepting *any* scan. While /clock is still 0 the
        # >5 s sanity check below is meaningless (0 ± 5 contains nothing
        # plausible), so the only safe option is to wait. A real
        # robot's wall clock is always >0, so this is a no-op there.
        scan_t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        node_t = self._now_s()
        if self._sim and node_t <= 0.0:
            return
        if node_t > 0.0 and abs(scan_t - node_t) > 5.0:
            with self._buf_lock:
                self._buf.clear()
            return

        # Monotonic-scan guard. Gazebo's /clock can advance during world
        # asset loading and then resume from a much smaller value once
        # the simulation actually starts running; the resulting backwards
        # jump in `scan.header.stamp` would poison the sweep accumulator
        # (mixing pre- and post-jump beams). Detect a >1 s backwards
        # jump, drop the offending scan, and clear the sweep buffer so
        # the next monotonically increasing scan starts fresh.
        #
        # IMPORTANT: do NOT reset `_last_published_stamp` here. MOLA-LO
        # already has the pre-jump (large) stamp in its rate-calc
        # buffer; if we let `sweep_start_s` reset to a small value, MOLA
        # will log "LiDAR timestamps went backwards" indefinitely (its
        # reference never updates because the comparison fails). The
        # publish-side bump guard in `_flush_sweep` instead forces the
        # next cloud's stamp to be 1 ms past the previous one, keeping
        # MOLA happy across the jump. Only `_last_flush_t` (which lives
        # in the same time domain as `scan.header.stamp`) is reset.
        if (self._last_scan_t > 0.0
                and scan_t < self._last_scan_t - 1.0):
            self.get_logger().warn(
                f'scan stamp jumped backwards ({self._last_scan_t:.3f} → '
                f'{scan_t:.3f} s); resetting sweep buffer.')
            with self._buf_lock:
                self._buf.clear()
            self._last_flush_t = 0.0
            self._last_scan_t = scan_t
            return
        self._last_scan_t = scan_t

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        # The LD06's own range_min can be as low as ~0.05 m, which means a
        # tilted beam grazing the chassis registers as a valid short return
        # and ends up below the floor in the output frame. Apply our own,
        # larger min_range on top of the sensor's nominal range_min.
        eff_min = max(scan.range_min, self._min_range)
        valid = (np.isfinite(ranges)
                 & (ranges >= eff_min)
                 & (ranges <= scan.range_max))
        if not np.any(valid):
            return

        # Per-beam timestamps.
        #
        # Three cases, in priority order:
        #   1. Driver reports time_increment > 0 AND scan_time > 0
        #      (current LD06 driver after the lipkg.cpp fix; also any
        #      proper Velodyne/Ouster/Hesai driver). The header stamp
        #      is the scan *start*, so beam i is at stamp + i*dt.
        #   2. Driver reports time_increment > 0 but doesn't tell us
        #      whether the stamp is start- or end-of-scan. We assume
        #      end-of-scan (the LD06 driver's historical behaviour
        #      before the fix) and reconstruct backwards.
        #   3. Driver reports time_increment == 0 (Gazebo gpu_lidar,
        #      or an unmodified upstream LD06 driver). Fall back to
        #      the configured `assumed_scan_period_s`: spread beams
        #      uniformly across that window ending at the header
        #      stamp. This is what stops a tilted-sweep cloud from
        #      stamping every beam in a 100 ms revolution at one
        #      single tilt angle (the "ladder streaks between walls"
        #      symptom on the real robot).
        scan_stamp_s = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        dt = float(scan.time_increment) if scan.time_increment > 0.0 else 0.0
        scan_time = float(scan.scan_time) if scan.scan_time > 0.0 else 0.0
        beam_idx_all = np.arange(n, dtype=np.float64)
        if dt > 0.0 and scan_time > 0.0:
            # Case 1: stamp is scan-start.
            beam_t_all = scan_stamp_s + beam_idx_all * dt
        elif dt > 0.0:
            # Case 2: stamp is scan-end (legacy LD06 behaviour).
            beam_t_all = scan_stamp_s - (n - 1 - beam_idx_all) * dt
        elif self._assumed_scan_period > 0.0:
            # Case 3: synthesise a uniform spread across the assumed
            # revolution period, anchored at the header stamp as
            # scan-end (the more conservative assumption — matches
            # what gpu_lidar / unmodified LD06 drivers do).
            dt_assumed = self._assumed_scan_period / max(n - 1, 1)
            beam_t_all = scan_stamp_s - (n - 1 - beam_idx_all) * dt_assumed
        else:
            beam_t_all = np.full(n, scan_stamp_s, dtype=np.float64)

        # Apply the validity mask.
        idx = beam_idx_all[valid]
        ranges = ranges[valid]
        theta = scan.angle_min + idx * scan.angle_increment
        beam_t = beam_t_all[valid]

        if scan.intensities and len(scan.intensities) == n:
            intensity = np.asarray(scan.intensities, dtype=np.float32)[valid]
        else:
            intensity = np.zeros(ranges.size, dtype=np.float32)

        # Per-beam tilt angle.
        tilt = self._tilt_for_beams(beam_t)
        if tilt is None:
            return  # real robot, no UART samples yet

        # Standard 2-D projection in the laser frame.
        x_l = ranges * np.cos(theta)
        y_l = ranges * np.sin(theta)

        # laser → lidar_mount_link  (Rz(π) + (0,0,sensor_z_offset))
        qx = -x_l
        qy = -y_l
        qz = self._z_off  # scalar; broadcasts

        # lidar_mount_link → base_link at angle `tilt` about base_link X
        # (URDF cancels upper_level_link's -π/2 yaw, so the joint axis is
        # exactly base_link X). Then translate by the hinge offset.
        cos_t = np.cos(tilt)
        sin_t = np.sin(tilt)
        x = (qx + self._hinge[0]).astype(np.float32)
        y = (qy * cos_t - qz * sin_t + self._hinge[1]).astype(np.float32)
        z = (qy * sin_t + qz * cos_t + self._hinge[2]).astype(np.float32)

        # Z floor/ceiling cull. Tilt-angle latency between /joint_states
        # (50 Hz) and /scan beams (10 Hz), plus the LD06's noisy near-
        # range returns, occasionally project floor hits a few cm below
        # ground in `base_link`. Those points (a) confuse MOLA-LO's
        # ground-plane estimate (driving Z drift in the robot pose) and
        # (b) show up as visible artefacts under the floor in RViz. Drop
        # them here; legitimate floor hits land at z ≈ 0 + noise and
        # survive the small negative threshold.
        z_mask = (z >= self._min_z) & (z <= self._max_z)
        if not z_mask.all():
            x = x[z_mask]
            y = y[z_mask]
            z = z[z_mask]
            intensity = intensity[z_mask]
            beam_t = beam_t[z_mask]
        if x.size == 0:
            return

        with self._buf_lock:
            self._buf.append((x, y, z, intensity, beam_t.astype(np.float64)))
            self._n_revs_in_sweep += 1

        # Drive the flush from here so it can never fire on a stale buffer
        # (which would re-stamp a cloud with `latest_t` already published).
        # `beam_t[-1]` is the freshest beam time we just appended.
        latest_beam_t = float(beam_t[-1])
        if (self._last_flush_t == 0.0
                or latest_beam_t - self._last_flush_t >= self._publish_period):
            self._last_flush_t = latest_beam_t
            self._flush_sweep()

    def _tilt_for_beams(self, beam_t: np.ndarray) -> np.ndarray | None:
        """Per-beam tilt angle by interpolation, or None if unavailable.

        Source of truth:
          - Real robot: UART angle samples pushed by `UartAngleReader`.
          - Sim: `/joint_states` samples pushed by `_on_joint_state`.
        Both populate the same `AngleHistory`, so this function doesn't
        need to know which mode it's running in.
        """
        snap = self._angles.snapshot()
        if snap is not None:
            t_hist, ang_hist = snap
            return np.interp(beam_t, t_hist, ang_hist).astype(np.float64)
        if self._sim:
            # No /joint_states yet; evaluate the commanded triangle
            # directly so we don't drop the very first scans.
            return np.fromiter(
                (_triangle(t - self._t0, self._sweep_period, self._lo, self._hi)
                 for t in beam_t),
                dtype=np.float64, count=beam_t.size,
            )
        return None  # real robot but no UART samples yet

    # ── Sweep flush: merge buffered points (last sweep_period_s) and publish
    def _flush_sweep(self) -> None:
        # Snapshot+evict under the lock so a concurrent `_on_scan` append
        # cannot race the eviction or the concatenate.
        with self._buf_lock:
            if not self._buf:
                return
            # Evict revolutions older than the sweep window. We use the most
            # recent beam time as 'now' so this works for both live and
            # bagged clocks without depending on get_clock().
            #
            # NOTE: comparison is `<=`, not `<`. Gazebo's gpu_lidar (and
            # any sensor with time_increment == 0) gives every beam in a
            # revolution the *same* timestamp. With `<` and
            # publish_period_s == sweep_period_s, the rev whose stamp
            # equals (latest_t - sweep_period) is never evicted, so the
            # next flush re-includes it and `sweep_start_s` matches the
            # previous flush's stamp → strict-monotonic guard drops the
            # whole cloud and MOLA sees a gap ("Not able to use velocity
            # motion model" / occasional "timestamps went backwards").
            latest_t = float(self._buf[-1][4][-1])
            cutoff = latest_t - self._sweep_period
            while self._buf and float(self._buf[0][4][-1]) <= cutoff:
                self._buf.popleft()
            if not self._buf:
                return
            revs_snapshot = list(self._buf)
            self._n_revs_in_sweep = 0

        x = np.concatenate([r[0] for r in revs_snapshot])
        y = np.concatenate([r[1] for r in revs_snapshot])
        z = np.concatenate([r[2] for r in revs_snapshot])
        intensity = np.concatenate([r[3] for r in revs_snapshot])
        t_abs = np.concatenate([r[4] for r in revs_snapshot])
        revs = len(revs_snapshot)

        # Drop NaN/inf points before publishing. MOLA's GICP and the MolaViz
        # renderer both trust `is_dense=True` and segfault if any non-finite
        # values sneak through.
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if not finite.all():
            x = x[finite]
            y = y[finite]
            z = z[finite]
            intensity = intensity[finite]
            t_abs = t_abs[finite]
        if x.size == 0:
            return

        # Stamp the cloud at the *oldest* beam (sweep start) and use
        # non-negative per-point `time` offsets. This matches the
        # convention every mainstream LiDAR driver (Velodyne / Ouster /
        # Hesai) uses and that MOLA-LO's online de-skewer assumes:
        # absolute beam time = header.stamp + time, with time >= 0.
        # Stamping at sweep_end with negative `time` made MOLA query its
        # navstate buffer for poses *before* the cloud stamp, which on
        # live clocks (where the buffer is aggressively trimmed) trips
        # "observation timestamps went backwards" within seconds.
        sweep_start_s = float(t_abs[0])

        # Strict-monotonic guard. MOLA-LO requires header stamps that
        # advance strictly between consecutive observations. If two
        # back-to-back flushes happen to compute the same `sweep_start_s`
        # — e.g. an upstream sim-clock tick is exactly aligned with
        # `publish_period_s`, or a duplicate /scan stamp leaks through —
        # we *bump* the new stamp by 1 ms instead of dropping the whole
        # cloud. Dropping starves MOLA's velocity model and produces the
        # "Not able to use velocity motion model for this timestep"
        # warnings.
        if sweep_start_s <= self._last_published_stamp:
            sweep_start_s = self._last_published_stamp + 1e-3
        self._last_published_stamp = sweep_start_s

        pts = np.empty(x.size, dtype=_POINT_DTYPE)
        pts['x'] = x
        pts['y'] = y
        pts['z'] = z
        pts['intensity'] = intensity
        # Per-point time = beam capture time relative to the cloud stamp
        # (sweep_start_s). Oldest beam is 0; newest beam is roughly
        # `sweep_period_s`. Clipped to >= 0 so that, if the bump above
        # pushed sweep_start_s past the oldest beam, those beams report
        # time = 0 (effectively de-skewed to the cloud reference time)
        # rather than a negative offset that would crash MOLA's buffer.
        pts['time'] = np.clip(
            (t_abs - sweep_start_s).astype(np.float32), 0.0, None)
        pts['ring'] = 0
        pts['_pad'] = 0

        cloud = PointCloud2()
        sec = int(sweep_start_s)
        cloud.header.stamp.sec = sec
        cloud.header.stamp.nanosec = int((sweep_start_s - sec) * 1e9)
        cloud.header.frame_id = self._out_frame
        cloud.height = 1
        cloud.width = pts.size
        cloud.fields = _FIELDS
        cloud.is_bigendian = False
        cloud.point_step = _POINT_STEP
        cloud.row_step = _POINT_STEP * pts.size
        cloud.is_dense = True
        cloud.data = pts.tobytes()
        self._cloud_pub.publish(cloud)
        self._n_clouds_pub += 1
        self.get_logger().debug(
            f'sweep flushed: {revs} revs, {pts.size} pts in {self._out_frame}')

    # ── Shutdown ──────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self._uart.stop()
        except Exception:  # noqa: BLE001
            pass
        if self._pwm is not None:
            try:
                self._pwm.change_duty_cycle(self.PWM_CENTER_PCT)
                self._pwm.stop()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
