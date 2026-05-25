#!/usr/bin/env python3
"""3D LiDAR node: tilt-servo sweep + sweep-accumulated PointCloud2. See wiki: Lidar3D/Overview."""

from __future__ import annotations

# --- Imports ---
# stdlib
import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import Deque, Tuple

# third-party
import numpy as np
import serial

# ROS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float64

# --- UART packet layout (little-endian, 7 bytes) ---
# [0]    sync   0xAA
# [1]    seq    uint8
# [2..3] angle  int16, centi-degrees
# [4..5] wiper  uint16, millivolts (ignored here)
# [6]    crc8   poly 0x07, init 0x00, over bytes 1..5
SYNC_BYTE = 0xAA
UART_PACKET_LEN = 7
CRC8_POLYNOMIAL = 0x07

# --- PointCloud2 layout (x y z intensity time ring + 2-byte pad) ---
# `ring` is constant 0 (single tilted ring), but FAST-LIO / LIO-SAM / MOLA-LO require the field.
# _POINT_DTYPE includes an explicit `_pad` so itemsize == _POINT_STEP (24). Don't replace with
# sensor_msgs_py.create_cloud — it derives the step from _FIELDS only (22) and breaks consumers.
_FIELDS = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time',      offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
]
_POINT_STEP = 24
_POINT_DTYPE = np.dtype({
    'names':    ['x', 'y', 'z', 'intensity', 'time', 'ring', '_pad'],
    'formats':  ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2', '<u2'],
    'offsets':  [0, 4, 8, 12, 16, 20, 22],
    'itemsize': _POINT_STEP,
})


# --- Helpers ---
def _build_crc8_table(poly: int) -> tuple[int, ...]:
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return tuple(table)


# Precomputed table for the encoder protocol's CRC-8 (poly 0x07).
_CRC8_TABLE = _build_crc8_table(CRC8_POLYNOMIAL)


def _crc8(buf: bytes) -> int:
    # Table-driven CRC-8 — one XOR + index per byte instead of an 8-iteration shift loop.
    c = 0
    for b in buf:
        c = _CRC8_TABLE[c ^ b]
    return c


@dataclass(slots=True)
class RevolutionSlice:
    """One revolution's worth of projected beam columns, all in output_frame."""
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    intensity: np.ndarray
    beam_t: np.ndarray


def _triangle(t: float, period: float, lo: float, hi: float) -> float:
    if period <= 0.0:
        return 0.5 * (lo + hi)
    phase = (t % period) / period
    tri = 1.0 - abs(2.0 * phase - 1.0)
    return lo + tri * (hi - lo)


class AngleHistory:
    """Lock-protected ring buffer of (t, angle_rad) samples used for per-beam tilt interpolation."""

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
        with self._lock:
            if not self._buf:
                return None
            arr = np.fromiter(
                (v for pair in self._buf for v in pair),
                dtype=np.float64,
                count=2 * len(self._buf),
            ).reshape(-1, 2)
        return arr[:, 0], arr[:, 1]


class UartAngleReader(threading.Thread):
    """Reads framed packets from the ESP32 pot/encoder, pushes (t, angle) into AngleHistory."""

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
            except Exception:
                pass

    def run(self) -> None:
        try:
            self._ser = serial.Serial(self._port_name, self._baud, timeout=0.1)
        except Exception as e:
            self._log.error(f'UART open failed ({self._port_name}): {e}')
            return
        self._log.info(f'UART open: {self._port_name} @ {self._baud}')

        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(64)
            except Exception as e:
                self._log.warning(f'UART read error: {e}')
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            self._consume(buf)

    def _consume(self, buf: bytearray) -> None:
        while len(buf) >= UART_PACKET_LEN:
            if buf[0] != SYNC_BYTE:
                del buf[0]
                continue
            packet = bytes(buf[:UART_PACKET_LEN])
            if _crc8(packet[1:6]) != packet[6]:
                del buf[0]
                continue
            angle_cdeg = int.from_bytes(packet[2:4], 'little', signed=True)
            angle_rad = self._sign * math.radians(angle_cdeg / 100.0) + self._offset
            t_now = self._clock.now().nanoseconds * 1e-9
            self._history.push(t_now, angle_rad)
            del buf[:UART_PACKET_LEN]


class Lidar3D(Node):
    def __init__(self):
        super().__init__('lidar3d')
        self._declare_params()
        self._read_params()

        # --- Sweep + accumulator state ---
        self._t0 = self._now_s()
        self._cmd_angle_rad = 0.0
        self._angles = AngleHistory()
        self._n_clouds_pub = 0

        # Per-revolution buffer in output_frame. Old revs evicted on flush so each cloud
        # spans only the last sweep_period_s.
        self._buf: Deque[RevolutionSlice] = deque()
        self._buf_lock = threading.Lock()

        # Strict-monotonic guard on published stamps. See wiki: Lidar3D/MonotonicStamps.
        self._last_published_stamp = 0.0
        # Wall-clock-style throttle for scan-driven flushes; same time domain as beam stamps.
        self._last_flush_t = 0.0
        # Monotonic guard on accepted scan stamps. See wiki: Lidar3D/SimClock.
        self._last_scan_t = 0.0

        self._init_pwm()

        # --- UART angle reader ---
        self._uart = UartAngleReader(
            port=self._uart_port, baud=self._uart_baud, history=self._angles,
            angle_offset_rad=self._angle_offset, invert=self._angle_invert,
            clock=self.get_clock(), logger=self.get_logger())
        if self._sim:
            self.get_logger().info('sim=True → UART angle reader disabled.')
        else:
            self._uart.start()

        # --- ROS interface ---
        self._cmd_pub = self.create_publisher(Float64, self._cmd_topic, 10)
        self._js_pub = self.create_publisher(JointState, self._js_topic, 10)
        self._cloud_pub = self.create_publisher(PointCloud2, self._cloud_topic, 5)

        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, sensor_qos)

        # In sim: subscribe to Gazebo's /joint_states bridge for measured tilt (UART is disabled).
        if self._sim:
            self.create_subscription(JointState, self._js_in_topic, self._on_joint_state, 50)

        # --- Timers ---
        self.create_timer(1.0 / self._pwm_hz, self._tick_sweep)
        self.create_timer(1.0 / self._js_hz, self._tick_joint_state)
        # No separate flush timer; _flush_sweep is driven by _on_scan. See wiki: Lidar3D/FlushTiming.
        self.create_timer(5.0, self._log_health)

        self.get_logger().info(
            f'lidar3d ready: sweep {math.degrees(self._lo):+.1f}…'
            f'{math.degrees(self._hi):+.1f} deg every {self._sweep_period:.2f} s, '
            f'cloud="{self._cloud_topic}" (frame={self._out_frame}, '
            f'window={self._sweep_period:.2f}s, publish every {self._publish_period:.2f}s).')

    # --- Parameter setup ---
    def _declare_params(self) -> None:
        # Mode
        self.declare_parameter('sim', False)

        # Tilt sweep
        self.declare_parameter('min_angle_deg', -30.0)
        self.declare_parameter('max_angle_deg', 30.0)
        self.declare_parameter('sweep_period_s', 4.0)
        # Each cloud contains beams from the last sweep_period_s. Consecutive clouds
        # overlap when publish_period_s < sweep_period_s; equal for disjoint sweeps.
        self.declare_parameter('publish_period_s', 0.5)
        self.declare_parameter('pwm_update_hz', 50.0)
        self.declare_parameter('pwm_chip', 1)
        self.declare_parameter('pwm_channel', 0)
        # Servo calibration: 50 Hz PWM, measured on the unit.
        self.declare_parameter('pwm_center_pct', 7.2)
        self.declare_parameter('pwm_pct_per_deg', 5.0 / 90.0)

        # UART angle feedback (real robot only)
        self.declare_parameter('uart_port', '/dev/ttyAMA1')
        self.declare_parameter('uart_baud', 921600)
        self.declare_parameter('angle_offset_deg', 163.52)  # pot midpoint
        self.declare_parameter('angle_invert', False)

        # Topics / frames
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/lidar3d/points')
        self.declare_parameter('joint_state_topic', '/lidar_joint_states')
        # Sim uses Gazebo's /joint_states (bridged) for measured tilt; real uses UART.
        self.declare_parameter('joint_states_in_topic', '/joint_states')
        self.declare_parameter('cmd_topic', '/lidar_cmd_pos')
        self.declare_parameter('joint_name', 'lidar_joint')
        self.declare_parameter('joint_state_rate_hz', 50.0)

        # Geometry — base_link → tilt-axis hinge point, in metres. See wiki: Lidar3D/Projection.
        self.declare_parameter('hinge_xyz_m', [0.00015423, -0.007561, 0.09175145])
        # lidar_mount_link → laser link Z offset (along base_link Z at tilt=0).
        self.declare_parameter('sensor_z_offset_m', 0.00615)
        # Output frame must be a tilt-stable parent (we bake per-beam tilt into the points).
        self.declare_parameter('output_frame', 'base_link')
        # Drop beams shorter than this (after the LD06's own range_min). Suppresses chassis hits.
        self.declare_parameter('min_range_m', 0.10)
        # Per-beam timing fallback when the LaserScan driver reports time_increment=0.
        # See wiki: Lidar3D/PerBeamTiming. 0.0 disables the spread.
        self.declare_parameter('assumed_scan_period_s', 0.1)
        # Z floor/ceiling cull in output_frame. Default disables it (very wide bounds).
        # Tune in YAML once the floor's actual Z is known. See wiki: Lidar3D/ZCull.
        self.declare_parameter('min_z_m', -1000.0)
        self.declare_parameter('max_z_m',  1000.0)

    def _read_params(self) -> None:
        self._sim = bool(self.get_parameter('sim').value)

        self._lo = math.radians(float(self.get_parameter('min_angle_deg').value))
        self._hi = math.radians(float(self.get_parameter('max_angle_deg').value))
        self._sweep_period = float(self.get_parameter('sweep_period_s').value)
        self._publish_period = float(self.get_parameter('publish_period_s').value)
        self._pwm_hz = float(self.get_parameter('pwm_update_hz').value)
        self._pwm_chip = int(self.get_parameter('pwm_chip').value)
        self._pwm_channel = int(self.get_parameter('pwm_channel').value)
        self._pwm_center_pct = float(self.get_parameter('pwm_center_pct').value)
        self._pwm_pct_per_deg = float(self.get_parameter('pwm_pct_per_deg').value)

        self._uart_port = str(self.get_parameter('uart_port').value)
        self._uart_baud = int(self.get_parameter('uart_baud').value)
        self._angle_offset = math.radians(float(self.get_parameter('angle_offset_deg').value))
        self._angle_invert = bool(self.get_parameter('angle_invert').value)

        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._cloud_topic = str(self.get_parameter('cloud_topic').value)
        self._js_topic = str(self.get_parameter('joint_state_topic').value)
        self._js_in_topic = str(self.get_parameter('joint_states_in_topic').value)
        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._joint_name = str(self.get_parameter('joint_name').value)
        self._js_hz = float(self.get_parameter('joint_state_rate_hz').value)

        self._hinge = np.asarray(list(self.get_parameter('hinge_xyz_m').value), dtype=np.float64).reshape(3)
        self._z_off = float(self.get_parameter('sensor_z_offset_m').value)
        self._out_frame = str(self.get_parameter('output_frame').value)
        self._min_range = float(self.get_parameter('min_range_m').value)
        self._min_z = float(self.get_parameter('min_z_m').value)
        self._max_z = float(self.get_parameter('max_z_m').value)
        self._assumed_scan_period = float(self.get_parameter('assumed_scan_period_s').value)

    # --- Hardware init ---
    def _init_pwm(self) -> None:
        self._pwm = None
        if self._sim:
            self.get_logger().info('sim=True → hardware PWM disabled.')
            return
        try:
            # Unexport first so a previous crash doesn't leave the channel locked (EBUSY on re-export).
            unexport = f'/sys/class/pwm/pwmchip{self._pwm_chip}/unexport'
            try:
                with open(unexport, 'w') as f:
                    f.write(str(self._pwm_channel))
            except OSError:
                pass

            from rpi_hardware_pwm import HardwarePWM
            self._pwm = HardwarePWM(pwm_channel=self._pwm_channel, hz=50, chip=self._pwm_chip)
            self._pwm.start(0)
            self._pwm.change_duty_cycle(self._pwm_center_pct)
            self.get_logger().info(
                f'Hardware PWM ready (chip={self._pwm_chip}, chan={self._pwm_channel}, 50 Hz).')
        except Exception as e:
            self.get_logger().error(f'Hardware PWM unavailable ({e}); command-only mode.')
            self._pwm = None

    # --- Helpers ---
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _current_tilt_rad(self) -> float:
        latest = self._angles.latest()
        return latest[1] if latest is not None else self._cmd_angle_rad

    # --- Periodic tasks ---
    def _tick_sweep(self) -> None:
        angle = _triangle(self._now_s() - self._t0, self._sweep_period, self._lo, self._hi)
        self._cmd_angle_rad = angle
        self._cmd_pub.publish(Float64(data=angle))
        if self._pwm is not None:
            duty = self._pwm_center_pct + math.degrees(angle) * self._pwm_pct_per_deg
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
        self.get_logger().info(f'5s window: clouds={self._n_clouds_pub}  {angle_str}  cmd={cmd_deg:+.2f}°')
        self._n_clouds_pub = 0

    # --- Sim: /joint_states → measured tilt ---
    def _on_joint_state(self, msg: JointState) -> None:
        try:
            i = msg.name.index(self._joint_name)
        except ValueError:
            return
        if i >= len(msg.position):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self._now_s()
        self._angles.push(t, float(msg.position[i]))

    # --- /scan → sweep accumulator ---
    def _on_scan(self, scan: LaserScan) -> None:
        if self._cloud_pub.get_subscription_count() == 0:
            return

        n = len(scan.ranges)
        if n == 0:
            return

        # Reject scans whose stamp is way off node clock; see wiki: Lidar3D/SimClock.
        scan_t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        node_t = self._now_s()
        if self._sim and node_t <= 0.0:
            return
        if node_t > 0.0 and abs(scan_t - node_t) > 5.0:
            with self._buf_lock:
                self._buf.clear()
            return

        # Monotonic scan guard for Gazebo /clock backwards jumps. See wiki: Lidar3D/SimClock.
        # Do NOT reset _last_published_stamp — _flush_sweep's bump guard handles MOLA-side.
        if self._last_scan_t > 0.0 and scan_t < self._last_scan_t - 1.0:
            self.get_logger().warning(
                f'scan stamp jumped backwards ({self._last_scan_t:.3f} → {scan_t:.3f} s); resetting sweep buffer.')
            with self._buf_lock:
                self._buf.clear()
            self._last_flush_t = 0.0
            self._last_scan_t = scan_t
            return
        self._last_scan_t = scan_t

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        # Apply our own min_range on top of the sensor's nominal range_min to suppress chassis hits.
        eff_min = max(scan.range_min, self._min_range)
        valid = np.isfinite(ranges) & (ranges >= eff_min) & (ranges <= scan.range_max)
        if not np.any(valid):
            return

        # Per-beam timestamps — see wiki: Lidar3D/PerBeamTiming for the three driver cases.
        scan_stamp_s = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        dt = float(scan.time_increment) if scan.time_increment > 0.0 else 0.0
        scan_time = float(scan.scan_time) if scan.scan_time > 0.0 else 0.0
        beam_idx_all = np.arange(n, dtype=np.float64)
        if dt > 0.0 and scan_time > 0.0:
            beam_t_all = scan_stamp_s + beam_idx_all * dt  # stamp = scan-start
        elif dt > 0.0:
            beam_t_all = scan_stamp_s - (n - 1 - beam_idx_all) * dt  # stamp = scan-end (legacy LD06)
        elif self._assumed_scan_period > 0.0:
            dt_assumed = self._assumed_scan_period / max(n - 1, 1)
            beam_t_all = scan_stamp_s - (n - 1 - beam_idx_all) * dt_assumed
        else:
            beam_t_all = np.full(n, scan_stamp_s, dtype=np.float64)

        idx = beam_idx_all[valid]
        ranges = ranges[valid]
        theta = scan.angle_min + idx * scan.angle_increment
        beam_t = beam_t_all[valid]

        if scan.intensities and len(scan.intensities) == n:
            intensity = np.asarray(scan.intensities, dtype=np.float32)[valid]
        else:
            intensity = np.zeros(ranges.size, dtype=np.float32)

        tilt = self._tilt_for_beams(beam_t)
        if tilt is None:
            return  # real robot, no UART samples yet

        # Standard 2-D projection in the laser frame.
        x_l = ranges * np.cos(theta)
        y_l = ranges * np.sin(theta)

        # laser → lidar_mount_link (Rz(π) + (0,0,sensor_z_offset)), then tilt about base_link X
        # and translate by hinge offset. See wiki: Lidar3D/Projection.
        qx = -x_l
        qy = -y_l
        qz = self._z_off
        cos_t = np.cos(tilt)
        sin_t = np.sin(tilt)
        x = (qx + self._hinge[0]).astype(np.float32)
        y = (qy * cos_t - qz * sin_t + self._hinge[1]).astype(np.float32)
        z = (qy * sin_t + qz * cos_t + self._hinge[2]).astype(np.float32)

        # Z floor/ceiling cull — drops sub-floor artefacts from tilt latency. See wiki: Lidar3D/ZCull.
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
            self._buf.append(RevolutionSlice(x, y, z, intensity, beam_t.astype(np.float64)))

        # Drive flush from here so it never fires on a stale buffer (would re-stamp with old latest_t).
        latest_beam_t = float(beam_t[-1])
        if self._last_flush_t == 0.0 or latest_beam_t - self._last_flush_t >= self._publish_period:
            self._last_flush_t = latest_beam_t
            self._flush_sweep()

    def _tilt_for_beams(self, beam_t: np.ndarray) -> np.ndarray | None:
        # Source of truth: real robot = UART samples; sim = /joint_states. Both feed AngleHistory.
        snap = self._angles.snapshot()
        if snap is not None:
            t_hist, ang_hist = snap
            return np.interp(beam_t, t_hist, ang_hist).astype(np.float64)
        if self._sim:
            # No /joint_states yet — evaluate the commanded triangle so we don't drop the first scans.
            return np.fromiter(
                (_triangle(t - self._t0, self._sweep_period, self._lo, self._hi)
                 for t in beam_t),
                dtype=np.float64, count=beam_t.size,
            )
        return None  # real robot, no UART samples yet

    # --- Sweep flush ---
    def _flush_sweep(self) -> None:
        # Snapshot+evict under lock so a concurrent _on_scan can't race the eviction or concat.
        with self._buf_lock:
            if not self._buf:
                return
            # Cutoff is <=, not <. See wiki: Lidar3D/SweepWindow (handles equal-stamp Gazebo gpu_lidar revs).
            latest_t = float(self._buf[-1].beam_t[-1])
            cutoff = latest_t - self._sweep_period
            while self._buf and float(self._buf[0].beam_t[-1]) <= cutoff:
                self._buf.popleft()
            if not self._buf:
                return
            revs_snapshot = list(self._buf)

        x = np.concatenate([r.x for r in revs_snapshot])
        y = np.concatenate([r.y for r in revs_snapshot])
        z = np.concatenate([r.z for r in revs_snapshot])
        intensity = np.concatenate([r.intensity for r in revs_snapshot])
        t_abs = np.concatenate([r.beam_t for r in revs_snapshot])

        # Drop non-finite points: MolaViz and MOLA's GICP segfault on NaN/inf with is_dense=True.
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if not finite.all():
            x = x[finite]
            y = y[finite]
            z = z[finite]
            intensity = intensity[finite]
            t_abs = t_abs[finite]
        if x.size == 0:
            return

        # Stamp the cloud at the oldest beam (sweep start). See wiki: Lidar3D/CloudStamping.
        sweep_start_s = float(t_abs[0])

        # Strict-monotonic guard: bump duplicate stamps by 1 ms instead of dropping the cloud.
        if sweep_start_s <= self._last_published_stamp:
            sweep_start_s = self._last_published_stamp + 1e-3
        self._last_published_stamp = sweep_start_s

        pts = np.empty(x.size, dtype=_POINT_DTYPE)
        pts['x'] = x
        pts['y'] = y
        pts['z'] = z
        pts['intensity'] = intensity
        # Per-point time = beam capture relative to sweep_start_s; clipped >=0 to survive the bump above.
        pts['time'] = np.clip((t_abs - sweep_start_s).astype(np.float32), 0.0, None)
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
            f'sweep flushed: {len(revs_snapshot)} revs, {pts.size} pts in {self._out_frame}')

    def destroy_node(self):
        try:
            self._uart.stop()
        except Exception:
            pass
        if self._pwm is not None:
            try:
                self._pwm.change_duty_cycle(self._pwm_center_pct)
                self._pwm.stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
