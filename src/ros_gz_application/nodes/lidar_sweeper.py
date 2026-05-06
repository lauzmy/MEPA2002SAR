#!/usr/bin/env python3
"""Step-and-settle tilt controller for the LD06 2-D LiDAR.

Instead of sweeping continuously (which leaves the servo permanently
"behind" its commanded angle and bobs the assembled cloud up and down),
this driver advances the servo in discrete steps:

    1. Command the next angle.
    2. Wait ``settle_time_s`` for the servo to physically arrive.
    3. Hold for ``hold_time_s`` -- long enough for the LD06 (10 Hz) to
       complete at least one full revolution.  Any LaserScan whose stamp
       falls inside this window has been captured at a known, *static*
       tilt angle.
    4. Repeat with the next angle in a triangle staircase.

Two topics let the assembler tell the two phases apart:

    /lidar_joint_states  (sensor_msgs/JointState, ~50 Hz)
        The currently commanded angle.  Drives ``robot_state_publisher``
        and TF.  During SETTLE the servo is mid-flight so this is briefly
        ahead of reality -- the assembler simply ignores scans during
        SETTLE, so TF is always queried while the servo is stationary.

    /lidar_hold_state    (std_msgs/Header, on every transition)
        ``frame_id`` is either ``"hold"`` or ``"settle"``.  ``stamp`` is
        the time the new phase began.  The assembler uses this to gate
        which scans it processes.

In simulation (``sim:=true``) the hardware PWM is skipped and the node
publishes only the command/state topics so the Gazebo bridge can drive
the joint.
"""

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header


def _build_staircase(min_rad: float, max_rad: float, step_rad: float) -> list[float]:
    """Triangle staircase from min->max->min, no duplicated endpoints."""
    if step_rad <= 0.0 or max_rad <= min_rad:
        return [0.0]
    n_up = max(1, int(round((max_rad - min_rad) / step_rad)))
    up = [min_rad + i * (max_rad - min_rad) / n_up for i in range(n_up + 1)]
    # Drop both endpoints on the way down so they aren't held twice in a row.
    down = list(reversed(up))[1:-1]
    return up + down


class LidarSweeper(Node):
    # --- Servo PWM calibration (50 Hz signal) ------------------------------
    # Measured: 4.7 % = -45 deg, 7.2 % = 0 deg, 9.7 % = +45 deg
    PWM_CENTER = 7.2                  # duty cycle (%) at neutral
    PWM_PERCENT_PER_DEG = 5.0 / 90.0  # duty-cycle delta per degree

    # /lidar_hold_state frame_id values
    PHASE_SETTLE = 'settle'
    PHASE_HOLD = 'hold'

    def __init__(self):
        super().__init__('lidar_sweeper')

        # ----- Parameters --------------------------------------------------
        self.declare_parameter('sim', False)
        self.declare_parameter('min_angle_deg', -30.0)
        self.declare_parameter('max_angle_deg', 30.0)
        self.declare_parameter('step_deg', 3.0)
        # Time allowed for the servo to physically reach the new angle.
        # Tune up until stationary objects stop bobbing in RViz.
        self.declare_parameter('settle_time_s', 0.08)
        # Time the servo is held still while scans are captured.  Must be
        # >= one LD06 revolution (100 ms @ 10 Hz) plus a small safety
        # margin so at least one full scan starts AND ends inside the
        # window.
        self.declare_parameter('hold_time_s', 0.15)
        self.declare_parameter('joint_state_rate_hz', 50.0)
        self.declare_parameter('joint_name', 'lidar_joint')
        self.declare_parameter('joint_state_topic', '/lidar_joint_states')
        self.declare_parameter('hold_state_topic', '/lidar_hold_state')
        self.declare_parameter('sweep_start_topic', '/lidar_sweep_start')
        self.declare_parameter('cmd_topic', '/lidar_cmd_pos')

        self.sim = bool(self.get_parameter('sim').value)
        a_min = math.radians(float(self.get_parameter('min_angle_deg').value))
        a_max = math.radians(float(self.get_parameter('max_angle_deg').value))
        step = math.radians(float(self.get_parameter('step_deg').value))
        self.settle_time = float(self.get_parameter('settle_time_s').value)
        self.hold_time = float(self.get_parameter('hold_time_s').value)
        self.joint_name = str(self.get_parameter('joint_name').value)
        js_topic = str(self.get_parameter('joint_state_topic').value)
        hold_topic = str(self.get_parameter('hold_state_topic').value)
        sweep_topic = str(self.get_parameter('sweep_start_topic').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)
        js_rate = float(self.get_parameter('joint_state_rate_hz').value)

        self._angles = _build_staircase(a_min, a_max, step)
        self._idx = 0
        self._phase = self.PHASE_SETTLE
        self._phase_t0 = self._now_s()

        # ----- Publishers --------------------------------------------------
        self.cmd_pub = self.create_publisher(Float64, cmd_topic, 10)
        self.js_pub = self.create_publisher(JointState, js_topic, 10)
        # Latch-style QoS would be nicer but Header doesn't justify a
        # custom QoS profile; just publish on every transition.
        self.hold_pub = self.create_publisher(Header, hold_topic, 10)
        # /lidar_sweep_start fires once at the moment the staircase wraps
        # back to its first step.  The assembler uses this to flush its
        # accumulator and publish a dense PointCloud2 per sweep.
        self.sweep_pub = self.create_publisher(Header, sweep_topic, 10)

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
                self.get_logger().error(
                    f'Could not init hardware PWM ({e}); running command-only.'
                )
                self.pwm = None
        else:
            self.get_logger().info('Simulation mode - PWM disabled')

        # Command the first angle and announce the initial settle phase.
        self._command_servo(self._angles[self._idx])
        self._publish_phase(self.PHASE_SETTLE)
        # Emit a sweep_start so the assembler has a defined accumulation
        # origin from the very first step.
        h0 = Header()
        h0.stamp = self.get_clock().now().to_msg()
        h0.frame_id = 'sweep_start'
        self.sweep_pub.publish(h0)

        # ----- Timers ------------------------------------------------------
        # Phase state machine runs much faster than the phase durations so
        # the transition timing is accurate.
        self._state_timer = self.create_timer(0.005, self._state_tick)
        # Continuous JointState publish keeps TF fresh for RViz / SLAM.
        self._js_timer = self.create_timer(1.0 / js_rate, self._publish_js)

        self.get_logger().info(
            f'Sweep staircase: {len(self._angles)} steps '
            f'({math.degrees(a_min):+.1f} to {math.degrees(a_max):+.1f} deg, '
            f'step {math.degrees(step):.2f} deg), '
            f'settle={self.settle_time*1000:.0f} ms, '
            f'hold={self.hold_time*1000:.0f} ms, '
            f'cycle={(self.settle_time+self.hold_time)*len(self._angles):.2f} s.'
        )

    # ------------------------------------------------------------------
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _state_tick(self) -> None:
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
        next_idx = (self._idx + 1) % len(self._angles)
        self._idx = next_idx
        self._command_servo(self._angles[self._idx])
        self._phase = self.PHASE_SETTLE
        self._phase_t0 = self._now_s()
        self._publish_phase(self.PHASE_SETTLE)
        # Wrapping back to the first step marks the start of a new sweep.
        if next_idx == 0:
            h = Header()
            h.stamp = self.get_clock().now().to_msg()
            h.frame_id = 'sweep_start'
            self.sweep_pub.publish(h)

    # ------------------------------------------------------------------
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
