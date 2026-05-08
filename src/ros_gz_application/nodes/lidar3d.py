#!/usr/bin/env python3
"""3D LiDAR node: tilt sweep + UART angle feedback + cloud assembly.

This single node owns everything related to turning the 2-D LD06
LaserScan into a 3-D PointCloud2 stream suitable for MOLA-LO:

    * Drives the tilt servo with a continuous triangle sweep (no
      stepping, no settle/hold gating).
    * Reads the *measured* tilt angle from an ESP32 over UART
      (/dev/ttyAMA1, 921600 8N1, framed binary packets).  Because we
      know the real angle at any time we don't have to trust the servo
      command.
    * For every incoming /scan we look up the measured tilt angle at
      each beam's timestamp, rotate the beam into the fixed mount
      frame, and publish the result as a PointCloud2 with a per-point
      `time` channel (the layout MOLA-LO / FAST-LIO expect).
    * Publishes /lidar_joint_states (the measured angle) so RViz and
      robot_state_publisher show the laser tilted correctly.

Frames
------
    body_link              fixed mount of the tilt mechanism
      └── lidar_joint      revolute (about Y)  -- angle from UART
            └── lidar_hinge_link
                  └── (fixed +Z offset)
                        └── laser              LD06 frame (/scan)

Output cloud is published in `body_link` with the tilt + Z offset
already baked into the points -- consumers don't need TF for the tilt.

UART packet (little-endian, 7 bytes)
------------------------------------
    [0]   sync     0xAA
    [1]   seq      uint8
    [2..3] angle   int16, centi-degrees (0..27000 spans the pot)
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
from std_msgs.msg import Float64, Header


# ── PointCloud2 layout (x y z intensity time ring) ───────────────────
# `ring` is constant 0 (single tilted ring), but FAST-LIO / LIO-SAM /
# Point-LIO all expect the field to exist -- it costs 2 bytes/point and
# guarantees portability.
_FIELDS = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time',      offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
]
_POINT_STEP = 24  # 5*float32 + uint16 + 2 bytes pad


def _crc8(buf: bytes) -> int:
    """CRC8 poly 0x07, init 0x00 -- matches the ESP32 firmware."""
    c = 0
    for b in buf:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


# ── Angle history: thread-safe ring buffer of (t_seconds, angle_rad) ───────
class AngleHistory:
    """Tiny lock-protected buffer for interpolating the measured tilt."""

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


# ── UART reader thread ─────────────────────────────────────────────────────
class UartAngleReader(threading.Thread):
    """Background thread: reads framed packets, pushes (t, angle) samples."""

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
        """Pull every complete, CRC-valid packet out of `buf`."""
        while len(buf) >= self.PACKET_LEN:
            if buf[0] != self.SYNC:
                del buf[0]
                continue
            packet = bytes(buf[:self.PACKET_LEN])
            if _crc8(packet[1:6]) != packet[6]:
                # Bad CRC -- drop the sync byte and resync.
                del buf[0]
                continue
            angle_cdeg = int.from_bytes(packet[2:4], 'little', signed=True)
            angle_rad = self._sign * math.radians(angle_cdeg / 100.0) + self._offset
            t_now = self._clock.now().nanoseconds * 1e-9
            self._history.push(t_now, angle_rad)
            del buf[:self.PACKET_LEN]


# ── Tilt sweep waveform ────────────────────────────────────────────────────
def _triangle(t: float, period: float, lo: float, hi: float) -> float:
    """Continuous triangle wave between `lo` and `hi` with period `period`."""
    if period <= 0.0:
        return 0.5 * (lo + hi)
    phase = (t % period) / period           # 0 .. 1
    tri = 1.0 - abs(2.0 * phase - 1.0)      # 0 .. 1 .. 0
    return lo + tri * (hi - lo)


# ── The node ───────────────────────────────────────────────────────────────
class Lidar3D(Node):
    # Servo PWM calibration (50 Hz signal; measured on the unit).
    PWM_CENTER_PCT = 7.2
    PWM_PCT_PER_DEG = 5.0 / 90.0

    def __init__(self):
        super().__init__('lidar3d')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('sim', False)

        # Sweep
        self.declare_parameter('min_angle_deg', -30.0)
        self.declare_parameter('max_angle_deg', 30.0)
        self.declare_parameter('sweep_period_s', 4.0)
        self.declare_parameter('pwm_update_hz', 50.0)
        self.declare_parameter('pwm_chip', 1)     # 1 on Pi 5 (pwmchip0 is RP1 internal)
        self.declare_parameter('pwm_channel', 0)  # GPIO12=ch0, GPIO13=ch1

        # UART angle feedback
        self.declare_parameter('uart_port', '/dev/ttyAMA1')
        self.declare_parameter('uart_baud', 921600)
        self.declare_parameter('angle_offset_deg', -163.52)  # pot midpoint
        self.declare_parameter('angle_invert', False)

        # Topics / frames
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/lidar3d/points')
        self.declare_parameter('joint_state_topic', '/lidar_joint_states')
        self.declare_parameter('cmd_topic', '/lidar_cmd_pos')
        self.declare_parameter('joint_name', 'lidar_joint')
        self.declare_parameter('output_frame', 'body_link')
        self.declare_parameter('sensor_z_offset_m', 0.032)  # hinge -> laser
        self.declare_parameter('joint_state_rate_hz', 50.0)

        sim = bool(self.get_parameter('sim').value)
        self._lo = math.radians(float(self.get_parameter('min_angle_deg').value))
        self._hi = math.radians(float(self.get_parameter('max_angle_deg').value))
        self._sweep_period = float(self.get_parameter('sweep_period_s').value)
        pwm_hz = float(self.get_parameter('pwm_update_hz').value)
        js_hz = float(self.get_parameter('joint_state_rate_hz').value)

        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._cloud_topic = str(self.get_parameter('cloud_topic').value)
        self._js_topic = str(self.get_parameter('joint_state_topic').value)
        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._joint_name = str(self.get_parameter('joint_name').value)
        self._out_frame = str(self.get_parameter('output_frame').value)
        self._z_off = float(self.get_parameter('sensor_z_offset_m').value)

        uart_port = str(self.get_parameter('uart_port').value)
        uart_baud = int(self.get_parameter('uart_baud').value)
        angle_offset = math.radians(float(self.get_parameter('angle_offset_deg').value))
        invert = bool(self.get_parameter('angle_invert').value)

        # ── State ─────────────────────────────────────────────────────────
        self._t0 = self._now_s()
        self._cmd_angle_rad = 0.0
        self._angles = AngleHistory()

        # ── Hardware PWM ──────────────────────────────────────────────────
        pwm_chip = int(self.get_parameter('pwm_chip').value)
        pwm_channel = int(self.get_parameter('pwm_channel').value)
        self._pwm = None
        if not sim:
            try:
                # Unexport first so a previous crash doesn't leave the channel
                # locked (kernel returns EBUSY on re-export otherwise).
                unexport = (f'/sys/class/pwm/pwmchip{pwm_chip}/unexport')
                try:
                    with open(unexport, 'w') as f:
                        f.write(str(pwm_channel))
                except OSError:
                    pass  # not exported yet -- that's fine

                from rpi_hardware_pwm import HardwarePWM
                self._pwm = HardwarePWM(pwm_channel=pwm_channel, hz=50, chip=pwm_chip)
                self._pwm.start(0)
                self._pwm.change_duty_cycle(self.PWM_CENTER_PCT)
                self.get_logger().info(
                    f'Hardware PWM ready (chip={pwm_chip}, chan={pwm_channel}, 50 Hz).')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(
                    f'Hardware PWM unavailable ({e}); running in command-only mode.')
                self._pwm = None
        else:
            self.get_logger().info('sim=True -> hardware PWM disabled.')

        # ── UART reader thread ────────────────────────────────────────────
        self._uart = UartAngleReader(
            port=uart_port, baud=uart_baud, history=self._angles,
            angle_offset_rad=angle_offset, invert=invert,
            clock=self.get_clock(), logger=self.get_logger())
        self._uart.start()

        # ── Publishers / subscribers ──────────────────────────────────────
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

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0 / pwm_hz, self._tick_sweep)
        self.create_timer(1.0 / js_hz, self._tick_joint_state)
        self.create_timer(5.0, self._log_health)

        # Stats
        self._n_scans_pub = 0
        self._n_scans_drop_no_angle = 0

        self.get_logger().info(
            f'lidar3d ready: sweep {math.degrees(self._lo):+.1f}..'
            f'{math.degrees(self._hi):+.1f} deg every {self._sweep_period:.2f} s, '
            f'cloud="{self._cloud_topic}" in "{self._out_frame}".')

    # ── Helpers ───────────────────────────────────────────────────────────
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ── Continuous sweep ──────────────────────────────────────────────────
    def _tick_sweep(self) -> None:
        angle = _triangle(self._now_s() - self._t0,
                          self._sweep_period, self._lo, self._hi)
        self._cmd_angle_rad = angle
        self._cmd_pub.publish(Float64(data=angle))
        if self._pwm is not None:
            duty = self.PWM_CENTER_PCT + math.degrees(angle) * self.PWM_PCT_PER_DEG
            self._pwm.change_duty_cycle(duty)

    # ── /lidar_joint_states (measured angle, falls back to commanded) ─────
    def _tick_joint_state(self) -> None:
        latest = self._angles.latest()
        angle = latest[1] if latest is not None else self._cmd_angle_rad
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._joint_name]
        js.position = [angle]
        js.velocity = [0.0]
        self._js_pub.publish(js)

    # ── Scan -> tilted PointCloud2 ────────────────────────────────────────
    def _on_scan(self, scan: LaserScan) -> None:
        # Need at least two angle samples to interpolate.
        snap = self._angles.snapshot()
        if snap is None:
            self._n_scans_drop_no_angle += 1
            return
        t_hist, ang_hist = snap

        n = len(scan.ranges)
        if n == 0:
            return

        # LD06 stamps the frame at the *end* of the revolution. Reconstruct
        # per-beam timestamps backwards from there.
        scan_end_s = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        dt = scan.time_increment if scan.time_increment > 0.0 else (1.0 / 10.0) / n
        beam_idx = np.arange(n, dtype=np.float64)
        # Beam 0 is the oldest, beam n-1 is the newest -> dt apart.
        beam_t = scan_end_s - (n - 1 - beam_idx) * dt
        scan_start_s = float(beam_t[0])

        # Per-beam tilt angle by linear interpolation of the UART history.
        tilt = np.interp(beam_t, t_hist, ang_hist).astype(np.float64)

        # Range filtering.
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = (np.isfinite(ranges)
                 & (ranges >= scan.range_min)
                 & (ranges <= scan.range_max))
        if not np.any(valid):
            return

        ranges = ranges[valid]
        tilt = tilt[valid]
        t_off = (beam_t[valid] - scan_start_s).astype(np.float32)
        theta = (scan.angle_min + beam_idx[valid] * scan.angle_increment)

        if scan.intensities and len(scan.intensities) == n:
            intensity = np.asarray(scan.intensities, dtype=np.float32)[valid]
        else:
            intensity = np.zeros(ranges.size, dtype=np.float32)

        # Beam in laser frame: x = r cos th, y = r sin th, z = 0.
        x_l = ranges * np.cos(theta)
        y_l = ranges * np.sin(theta)

        # Rotate around Y by `tilt`, then translate +z_off (hinge -> laser).
        # [ x']   [ cos t  0  sin t ] [ x_l ]
        # [ y'] = [   0    1    0   ] [ y_l ]
        # [ z']   [-sin t  0  cos t ] [  0  ] + (0, 0, z_off)
        cos_t = np.cos(tilt)
        sin_t = np.sin(tilt)
        x = (x_l * cos_t).astype(np.float32)
        y = y_l.astype(np.float32)
        z = (-x_l * sin_t + self._z_off).astype(np.float32)

        # Pack into the PointCloud2 byte buffer.
        pts = np.empty(x.size, dtype=np.dtype({
            'names':    ['x', 'y', 'z', 'intensity', 'time', 'ring', '_pad'],
            'formats':  ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2', '<u2'],
            'offsets':  [0, 4, 8, 12, 16, 20, 22],
            'itemsize': _POINT_STEP,
        }))
        pts['x'] = x
        pts['y'] = y
        pts['z'] = z
        pts['intensity'] = intensity
        pts['time'] = t_off
        pts['ring'] = 0
        pts['_pad'] = 0

        cloud = PointCloud2()
        cloud.header = Header()
        # header.stamp = first beam (sweep start) -> per-point time is positive.
        sec = int(scan_start_s)
        cloud.header.stamp.sec = sec
        cloud.header.stamp.nanosec = int((scan_start_s - sec) * 1e9)
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
        self._n_scans_pub += 1

    # ── Diagnostics ───────────────────────────────────────────────────────
    def _log_health(self) -> None:
        latest = self._angles.latest()
        age_ms = -1.0
        ang_deg = float('nan')
        if latest is not None:
            age_ms = (self._now_s() - latest[0]) * 1e3
            ang_deg = math.degrees(latest[1])
        self.get_logger().info(
            f'5s window: clouds={self._n_scans_pub}  '
            f'dropped(no UART)={self._n_scans_drop_no_angle}  '
            f'angle={ang_deg:+.2f} deg ({age_ms:.0f} ms old)  '
            f'cmd={math.degrees(self._cmd_angle_rad):+.2f} deg')
        self._n_scans_pub = 0
        self._n_scans_drop_no_angle = 0

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
