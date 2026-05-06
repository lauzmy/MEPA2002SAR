#!/usr/bin/env python3
"""Step-and-settle tilt controller for the LD06 2-D LiDAR.

The servo advances in discrete steps rather than sweeping continuously so
every accepted LaserScan is captured while the servo is known to be still:

    1. Command the next staircase angle.
    2. Wait ``settle_time_s`` for the servo to physically arrive.
    3. Hold for ``hold_time_s`` (auto-derived from scan rate by default).
    4. Repeat through a triangle staircase: min → max → min.

Density / speed trade-off
--------------------------
Set ``quality_preset`` to one of:  fast (20 scans/sweep), balanced (50),
dense (100), or custom (use ``scans_per_sweep`` directly).
``step_deg > 0`` overrides everything and sets a fixed step size.

Auto-calibration
----------------
Pass ``auto_calibrate_on_start: true`` (or call the ``~/calibrate_settle``
service) to measure how long the servo actually takes to settle by watching
how quickly the floor-band returns stabilise after each commanded step.
The measured value × ``calibration_safety_factor`` replaces ``settle_time_s``
for the current session (and is logged so you can make it permanent).

Phase topic
-----------
Every phase transition is announced on ``/lidar_hold_state``
(std_msgs/Header) with ``frame_id`` set to:
  "hold"        – servo is stationary, scans are valid
  "settle"      – servo is moving, scans are invalid
  "calibrating" – calibration in progress, all scans should be dropped
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64, Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


# ---------------------------------------------------------------------------
QUALITY_PRESETS: dict[str, int] = {'fast': 20, 'balanced': 50, 'dense': 100}


def _build_staircase(min_rad: float, max_rad: float, n_up: int) -> list[float]:
    """Triangle staircase min→max→min with exactly ``2 * n_up`` stops."""
    n_up = max(1, n_up)
    up = [min_rad + i * (max_rad - min_rad) / n_up for i in range(n_up + 1)]
    down = list(reversed(up))[1:-1]   # drop both endpoints; visited on up pass
    return up + down


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
class LidarSweeper(Node):
    # --- Servo PWM calibration (50 Hz signal) ------------------------------
    # Measured: 4.7 % = -45 deg, 7.2 % = 0 deg, 9.7 % = +45 deg
    PWM_CENTER = 7.2                  # duty cycle (%) at neutral
    PWM_PERCENT_PER_DEG = 5.0 / 90.0  # duty-cycle delta per degree

    PHASE_SETTLE = 'settle'
    PHASE_HOLD = 'hold'
    PHASE_CALIBRATING = 'calibrating'

    def __init__(self):
        super().__init__('lidar_sweeper')

        # ----- Parameters --------------------------------------------------
        self.declare_parameter('sim', False)
        self.declare_parameter('min_angle_deg', -30.0)
        self.declare_parameter('max_angle_deg', 30.0)

        # Density / speed knob (priority: step_deg > scans_per_sweep > quality_preset)
        self.declare_parameter('quality_preset', 'balanced')   # fast|balanced|dense|custom
        self.declare_parameter('scans_per_sweep', 50)          # used when preset=custom
        self.declare_parameter('step_deg', -1.0)               # <= 0 = auto-compute

        # Timing
        self.declare_parameter('settle_time_s', 0.08)
        self.declare_parameter('hold_time_s', -1.0)            # <= 0 = auto-compute
        self.declare_parameter('lidar_scan_rate_hz', 10.0)
        self.declare_parameter('hold_margin_s', 0.02)
        self.declare_parameter('min_scans_per_stop', 1)

        # Topics / joints
        self.declare_parameter('joint_name', 'lidar_joint')
        self.declare_parameter('joint_state_topic', '/lidar_joint_states')
        self.declare_parameter('joint_state_rate_hz', 100.0)
        self.declare_parameter('hold_state_topic', '/lidar_hold_state')
        self.declare_parameter('cmd_topic', '/lidar_cmd_pos')
        self.declare_parameter('scan_topic', '/scan')

        # Calibration
        self.declare_parameter('auto_calibrate_on_start', False)
        self.declare_parameter('output_frame', 'base_footprint')
        self.declare_parameter('calibration_floor_band_m', 0.06)
        self.declare_parameter('calibration_tolerance_m', 0.005)
        self.declare_parameter('calibration_safety_factor', 1.3)
        self.declare_parameter('calibration_min_scans_stable', 3)

        # ----- Resolve parameters ------------------------------------------
        self.sim = bool(self.get_parameter('sim').value)
        a_min = math.radians(float(self.get_parameter('min_angle_deg').value))
        a_max = math.radians(float(self.get_parameter('max_angle_deg').value))

        preset = str(self.get_parameter('quality_preset').value)
        n_scans = QUALITY_PRESETS.get(preset, int(self.get_parameter('scans_per_sweep').value))
        n_up = max(1, n_scans // 2)

        step_deg_override = float(self.get_parameter('step_deg').value)
        if step_deg_override > 0.0:
            n_up = max(1, int(round((a_max - a_min) / math.radians(step_deg_override))))

        self.settle_time = float(self.get_parameter('settle_time_s').value)
        scan_rate = float(self.get_parameter('lidar_scan_rate_hz').value)
        hold_margin = float(self.get_parameter('hold_margin_s').value)
        min_scans = int(self.get_parameter('min_scans_per_stop').value)
        hold_param = float(self.get_parameter('hold_time_s').value)
        self.hold_time = hold_param if hold_param > 0.0 else min_scans / scan_rate + hold_margin

        self.joint_name = str(self.get_parameter('joint_name').value)
        js_topic = str(self.get_parameter('joint_state_topic').value)
        hold_topic = str(self.get_parameter('hold_state_topic').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        js_rate = float(self.get_parameter('joint_state_rate_hz').value)
        auto_cal = bool(self.get_parameter('auto_calibrate_on_start').value)

        self._output_frame = str(self.get_parameter('output_frame').value)
        self._floor_band = float(self.get_parameter('calibration_floor_band_m').value)
        self._cal_tol = float(self.get_parameter('calibration_tolerance_m').value)
        self._cal_safety = float(self.get_parameter('calibration_safety_factor').value)
        self._cal_min_stable = int(self.get_parameter('calibration_min_scans_stable').value)

        self._angles = _build_staircase(a_min, a_max, n_up)
        self._idx = 0
        self._phase = self.PHASE_SETTLE
        self._phase_t0 = self._now_s()
        self._calibrating = False

        step_actual = (a_max - a_min) / n_up
        n_stops = len(self._angles)
        cycle_s = n_stops * (self.settle_time + self.hold_time)
        self.get_logger().info(
            f'Sweep plan: {n_stops} stops, step={math.degrees(step_actual):.2f} deg, '
            f'settle={self.settle_time*1000:.0f} ms, hold={self.hold_time*1000:.0f} ms, '
            f'cycle={cycle_s:.1f} s  [preset={preset}]'
        )

        # ----- Publishers --------------------------------------------------
        self.cmd_pub = self.create_publisher(Float64, cmd_topic, 10)
        self.js_pub = self.create_publisher(JointState, js_topic, 10)
        self.hold_pub = self.create_publisher(Header, hold_topic, 10)

        # ----- Hardware PWM (real robot only) ------------------------------
        self.pwm = None
        if not self.sim:
            try:
                from rpi_hardware_pwm import HardwarePWM
                self.pwm = HardwarePWM(pwm_channel=0, hz=50, chip=0)
                self.pwm.start(0)
                self.pwm.change_duty_cycle(self.PWM_CENTER)
                time.sleep(0.5)
                self.get_logger().info('Hardware PWM initialised (channel 0, 50 Hz)')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'Hardware PWM failed ({e}); command-only.')
        else:
            self.get_logger().info('Simulation mode - PWM disabled')

        # ----- Calibration scan buffer + TF --------------------------------
        # Scans are only recorded into this list when _cal_active is True.
        self._cal_scans: list[LaserScan] = []
        self._cal_active = False
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, scan_topic, self._cal_scan_cb, sensor_qos)
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ----- Calibration service -----------------------------------------
        self.create_service(Trigger, '~/calibrate_settle', self._calibrate_srv_cb)

        # ----- Start state machine -----------------------------------------
        self._command_servo(self._angles[self._idx])
        self._publish_phase(self.PHASE_SETTLE)
        self._state_timer = self.create_timer(0.005, self._state_tick)
        self._js_timer = self.create_timer(1.0 / js_rate, self._publish_js)

        # Calibration fires after spin() starts so callbacks work during sleep
        if auto_cal:
            self._calibrating = True
            self._publish_phase(self.PHASE_CALIBRATING)
            self._auto_cal_fired = False
            self.create_timer(0.5, self._auto_cal_timer)

    # ------------------------------------------------------------------
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ----- State machine --------------------------------------------------
    def _state_tick(self) -> None:
        if self._calibrating:
            return
        elapsed = self._now_s() - self._phase_t0
        if self._phase == self.PHASE_SETTLE and elapsed >= self.settle_time:
            self._enter_hold()
        elif self._phase == self.PHASE_HOLD and elapsed >= self.hold_time:
            self._enter_settle()

    def _enter_hold(self) -> None:
        self._phase = self.PHASE_HOLD
        self._phase_t0 = self._now_s()
        self._publish_phase(self.PHASE_HOLD)

    def _enter_settle(self) -> None:
        self._idx = (self._idx + 1) % len(self._angles)
        self._command_servo(self._angles[self._idx])
        self._phase = self.PHASE_SETTLE
        self._phase_t0 = self._now_s()
        self._publish_phase(self.PHASE_SETTLE)

    def _publish_phase(self, phase: str) -> None:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = phase
        self.hold_pub.publish(h)

    def _publish_js(self) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [self._angles[self._idx]]
        js.velocity = [0.0]
        self.js_pub.publish(js)

    def _command_servo(self, angle_rad: float) -> None:
        self.cmd_pub.publish(Float64(data=angle_rad))
        if self.pwm is not None:
            duty = self.PWM_CENTER + math.degrees(angle_rad) * self.PWM_PERCENT_PER_DEG
            self.pwm.change_duty_cycle(duty)

    # ----- Calibration ----------------------------------------------------
    def _cal_scan_cb(self, scan: LaserScan) -> None:
        if self._cal_active:
            self._cal_scans.append(scan)

    def _floor_z_from_scan(self, scan: LaserScan) -> 'float | None':
        """Return median z (output_frame) of floor-band points, or None."""
        n = len(scan.ranges)
        if n == 0:
            return None
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        if not np.any(valid):
            return None
        beam_idx = np.where(valid)[0].astype(np.float64)
        theta = scan.angle_min + beam_idx * scan.angle_increment
        r = ranges[valid]
        pts = np.stack([r * np.cos(theta), r * np.sin(theta), np.zeros(r.size)], axis=1)
        try:
            tf = self._tf_buf.lookup_transform(
                self._output_frame, scan.header.frame_id,
                Time.from_msg(scan.header.stamp), timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        pts_out = pts @ _quat_to_rot(q.x, q.y, q.z, q.w).T + np.array([t.x, t.y, t.z])
        floor = pts_out[(pts_out[:, 2] >= -self._floor_band) & (pts_out[:, 2] <= self._floor_band)]
        return float(np.median(floor[:, 2])) if len(floor) >= 5 else None

    def _measure_settle(self, start_rad: float, end_rad: float,
                        capture_s: float = 1.0) -> 'float | None':
        """Command start→end, collect scans for capture_s, return settle time."""
        self._command_servo(start_rad)
        time.sleep(0.5)                    # wait for servo to park at start
        t_cmd = self._now_s()
        self._cal_scans.clear()
        self._cal_active = True
        self._command_servo(end_rad)
        time.sleep(capture_s)              # collect scans in background callbacks
        self._cal_active = False

        scans = list(self._cal_scans)
        if len(scans) < self._cal_min_stable + 2:
            return None

        samples: list[tuple[float, float]] = []
        for sc in scans:
            t_rel = sc.header.stamp.sec + sc.header.stamp.nanosec * 1e-9 - t_cmd
            z = self._floor_z_from_scan(sc)
            if z is not None:
                samples.append((t_rel, z))

        if len(samples) < self._cal_min_stable + 2:
            return None

        t_arr = np.array([s[0] for s in samples])
        z_arr = np.array([s[1] for s in samples])
        z_final = float(np.median(z_arr[-5:]))

        for i in range(len(samples) - self._cal_min_stable):
            window = z_arr[i: i + self._cal_min_stable]
            if np.all(np.abs(window - z_final) <= self._cal_tol):
                return float(t_arr[i])
        return None  # did not settle within capture window

    def _run_calibration(self) -> None:
        a_min, a_max = self._angles[0], self._angles[len(self._angles) // 2]
        step = (a_max - a_min) / max(1, len(self._angles) // 2)
        tests = [
            (a_min, a_min + step,                          'single step'),
            (a_min, a_min + 2 * step,                      'double step'),
            (a_min, a_min + (a_max - a_min) * 0.5,        'half amplitude'),
        ]
        self.get_logger().info('--- Settle-time calibration starting ---')
        results: list[float] = []
        for start, end, label in tests:
            measured = self._measure_settle(start, end)
            if measured is not None:
                results.append(measured)
                self.get_logger().info(
                    f'  {label} ({math.degrees(end-start):.1f} deg) '
                    f'-> settled {measured*1000:.0f} ms after command')
            else:
                self.get_logger().warn(
                    f'  {label} -> no settling detected (floor band empty?)')

        if results:
            new_settle = min(0.50, max(0.04, max(results) * self._cal_safety))
            self.settle_time = new_settle
            self.get_logger().info(
                f'Calibration result: worst={max(results)*1000:.0f} ms × '
                f'{self._cal_safety} safety = {new_settle*1000:.0f} ms. '
                f'Set  settle_time_s: {new_settle:.3f}  in your launch file.')
        else:
            self.get_logger().error(
                f'Calibration failed: keeping settle_time_s={self.settle_time:.3f} s.')

    def _start_calibration_bg(self) -> None:
        """Launch calibration in a background thread so scan callbacks keep firing."""
        def _worker():
            self._calibrating = True
            self._publish_phase(self.PHASE_CALIBRATING)
            try:
                self._run_calibration()
            finally:
                self._calibrating = False
                self._phase = self.PHASE_SETTLE
                self._phase_t0 = self._now_s()
                self._command_servo(self._angles[self._idx])
                self._publish_phase(self.PHASE_SETTLE)
        threading.Thread(target=_worker, daemon=True).start()

    def _auto_cal_timer(self) -> None:
        if getattr(self, '_auto_cal_fired', False):
            return
        self._auto_cal_fired = True
        self._start_calibration_bg()

    def _calibrate_srv_cb(self, request, response):  # noqa: ARG002
        if self._calibrating:
            response.success = False
            response.message = 'Calibration already running'
            return response
        self._start_calibration_bg()
        response.success = True
        response.message = 'Calibration started; watch node logs for results.'
        return response

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.pwm is not None:
            try:
                self.get_logger().info('Centering servo and stopping PWM')
                self.pwm.change_duty_cycle(self.PWM_CENTER)
                time.sleep(0.5)
                self.pwm.stop()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarSweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
