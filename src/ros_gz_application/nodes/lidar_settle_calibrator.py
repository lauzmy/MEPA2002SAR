#!/usr/bin/env python3
"""Recommend a ``settle_time_s`` for ``lidar_sweeper`` (step-and-settle).

How it works
------------
For each test step size (degrees), the helper:

  1. Drives the servo to a "home" angle and waits ``home_settle_s``.
  2. Records a "reference" scan after a long settle -- this is the
     ground truth the servo has fully arrived.
  3. Steps the servo to the home angle (so we know it has to traverse
     ``step_deg`` from a known starting point), notes ``t_step``, and
     greedily collects every incoming scan for ``observe_window_s``.
  4. For each collected scan computes a similarity metric vs. the
     reference scan (median absolute range delta over beams that are
     valid in both).  Reports the time of the first scan whose metric
     is below ``tolerance_m`` *and* whose successors stay below it for
     at least ``stable_count`` consecutive scans.
  5. The recommended settle_time is the worst (max) over all step sizes,
     plus a small safety margin.

The helper drives the hardware PWM **directly**.  Use the launch file
so the LD06 driver is started for you and the regular ``lidar_sweeper``
is NOT running at the same time::

    ros2 launch ros_gz_bringup lidar_calibrate.launch.py
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import LaserScan


class SettleCalibrator(Node):
    PWM_CENTER = 7.2
    PWM_PERCENT_PER_DEG = 5.0 / 90.0

    def __init__(self):
        super().__init__('lidar_settle_calibrator')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('home_deg', 0.0)
        # Step sizes to test (small to large).  The largest step
        # dominates the recommended settle time.
        self.declare_parameter('test_steps_deg', [5.0, 10.0, 20.0, 30.0, 45.0])
        self.declare_parameter('home_settle_s', 1.0)
        self.declare_parameter('observe_window_s', 0.6)
        # |range_now - range_reference| median, in metres, considered
        # "settled" for a given beam.
        self.declare_parameter('tolerance_m', 0.02)
        # Number of consecutive scans the metric must stay below
        # tolerance for the servo to be considered settled.
        self.declare_parameter('stable_count', 2)
        # Margin added on top of the worst measured settle time.
        self.declare_parameter('safety_margin_s', 0.02)
        # Ignore very-far returns (beyond this) for the metric -- noisy
        # at the edge of the LD06 range.
        self.declare_parameter('max_range_m', 6.0)

        scan_topic = str(self.get_parameter('scan_topic').value)
        self.home_deg = float(self.get_parameter('home_deg').value)
        self.test_steps = [float(x) for x in
                           self.get_parameter('test_steps_deg').value]
        self.home_settle_s = float(self.get_parameter('home_settle_s').value)
        self.observe_window_s = float(self.get_parameter('observe_window_s').value)
        self.tol_m = float(self.get_parameter('tolerance_m').value)
        self.stable_count = int(self.get_parameter('stable_count').value)
        self.margin_s = float(self.get_parameter('safety_margin_s').value)
        self.max_range = float(self.get_parameter('max_range_m').value)

        # ----- Hardware PWM ------------------------------------------------
        try:
            from rpi_hardware_pwm import HardwarePWM
            self.pwm = HardwarePWM(pwm_channel=0, hz=50, chip=0)
            self.pwm.start(0)
            self.pwm.change_duty_cycle(self.PWM_CENTER)
            time.sleep(0.5)
            self.get_logger().info('PWM ready (channel 0, 50 Hz).')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Could not init PWM ({e}); aborting.')
            raise SystemExit(1) from e

        # ----- Scan subscription -------------------------------------------
        # Buffer of (t_recv_s, ranges) for the currently-recording window.
        self._buffer: list[tuple[float, np.ndarray]] = []
        self._recording = False
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor_qos)

    # ------------------------------------------------------------------
    def _command(self, angle_deg: float) -> None:
        duty = self.PWM_CENTER + angle_deg * self.PWM_PERCENT_PER_DEG
        self.pwm.change_duty_cycle(duty)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, scan: LaserScan) -> None:
        if not self._recording:
            return
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        # Clamp invalid / out-of-range to NaN so they don't pollute the metric
        bad = ~np.isfinite(ranges) | (ranges < scan.range_min) \
              | (ranges > min(scan.range_max, self.max_range))
        ranges[bad] = np.nan
        # Use scan.header.stamp for accuracy (stamp = end of scan on LD06).
        t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        self._buffer.append((t, ranges))

    # ------------------------------------------------------------------
    def _record_window(self, duration_s: float) -> list[tuple[float, np.ndarray]]:
        self._buffer = []
        self._recording = True
        t_end = self._now_s() + duration_s
        while self._now_s() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
        self._recording = False
        return list(self._buffer)

    @staticmethod
    def _delta(reference: np.ndarray, current: np.ndarray) -> float:
        """Median absolute range difference over beams valid in both."""
        both = np.isfinite(reference) & np.isfinite(current)
        if not np.any(both):
            return float('inf')
        return float(np.median(np.abs(reference[both] - current[both])))

    def _measure_step(self, step_deg: float) -> float | None:
        """Drive the servo home, then step by ``step_deg`` and return the
        measured settle time (seconds), or None if the test could not be
        evaluated.
        """
        target_deg = self.home_deg + step_deg
        self.get_logger().info(
            f'--- step {step_deg:+.1f} deg '
            f'(home {self.home_deg:+.1f} -> target {target_deg:+.1f}) ---')

        # 1. Park at target for a long time, take reference.
        self._command(target_deg)
        time.sleep(self.home_settle_s)
        ref_scans = self._record_window(0.3)
        if not ref_scans:
            self.get_logger().error(
                'Lost /scan stream during reference recording -- aborting step.')
            return None
        # Average the last few reference scans beam-wise (NaN-safe).
        ref_stack = np.vstack([r for _, r in ref_scans[-3:]])
        with np.errstate(all='ignore'):
            reference = np.nanmedian(ref_stack, axis=0)

        # 2. Park back at home for a long time so the servo is fully still.
        self._command(self.home_deg)
        time.sleep(self.home_settle_s)

        # 3. Step to target, record window, note step time precisely.
        # Pre-arm the buffer so we don't miss the first scan.
        self._buffer = []
        self._recording = True
        t_step = self._now_s()
        self._command(target_deg)
        # Spin until window done.
        t_end = t_step + self.observe_window_s
        while self._now_s() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
        self._recording = False
        observations = list(self._buffer)
        if not observations:
            self.get_logger().error('No scans observed during step window.')
            return None

        # 4. Compute deltas and find first stable run.
        deltas = [(t - t_step, self._delta(reference, r)) for t, r in observations]
        for i, (dt, d) in enumerate(deltas):
            self.get_logger().info(
                f'  t={dt*1000:6.1f} ms  delta={d*1000:6.1f} mm  '
                f'{"OK" if d <= self.tol_m else "  "}')

        # First index where delta and the next stable_count-1 are below tol.
        for i in range(len(deltas) - self.stable_count + 1):
            window = deltas[i:i + self.stable_count]
            if all(d <= self.tol_m for _, d in window):
                settle_t = window[0][0]
                self.get_logger().info(
                    f'  -> settled at t={settle_t*1000:.1f} ms.')
                return float(settle_t)

        self.get_logger().warn(
            f'  -> never settled within tolerance ({self.tol_m*1000:.0f} mm) '
            f'in {self.observe_window_s*1000:.0f} ms.')
        return None

    # ------------------------------------------------------------------
    def run(self) -> None:
        # Brief settle for the lidar driver to start streaming.  The
        # launch file brings the driver up before we are spawned, but
        # the first few scans can still be missing.
        deadline = self._now_s() + 2.0
        while self._now_s() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        results: dict[float, float] = {}
        for step_deg in self.test_steps:
            t = self._measure_step(step_deg)
            if t is not None:
                results[step_deg] = t

        # Park at home before exit.
        self._command(self.home_deg)
        time.sleep(0.3)

        if not results:
            self.get_logger().error('No measurements -- nothing to recommend.')
            return

        worst_step, worst_t = max(results.items(), key=lambda kv: kv[1])
        recommended = math.ceil((worst_t + self.margin_s) * 1000) / 1000.0
        self.get_logger().info('=' * 60)
        self.get_logger().info('Calibration summary:')
        for s, t in sorted(results.items()):
            self.get_logger().info(f'   step {s:+5.1f} deg -> settle {t*1000:6.1f} ms')
        self.get_logger().info(
            f'Worst case: {worst_step:+.1f} deg @ {worst_t*1000:.1f} ms '
            f'(+ {self.margin_s*1000:.0f} ms margin)')
        self.get_logger().info(
            f'==> Recommended settle_time_s: {recommended:.3f}')
        self.get_logger().info('=' * 60)

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            self.pwm.change_duty_cycle(self.PWM_CENTER)
            time.sleep(0.3)
            self.pwm.stop()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SettleCalibrator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
