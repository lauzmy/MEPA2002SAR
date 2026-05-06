#!/usr/bin/env python3
"""Tilt-servo controller for the LD06 2D-LiDAR.

Sweeps the lidar pitch with a *triangle* wave so the angular velocity is
constant.  The period is locked to an integer number of full LD06 scans
(10 Hz) so every sweep contains exactly the same set of scan stamps.

The instantaneous tilt angle is a deterministic function of ROS time:

    alpha(t) = triangle( (t - t0) / period ) * amplitude

Because both this node and the assembler use the same equation, no
message synchronisation between them is needed - the assembler can ask
"what was the tilt at the timestamp of this beam?" without any TF/topic
look-up.

The same angle is also published as a JointState (for robot_state_publisher
and RViz) and as a Float64 on /lidar_cmd_pos (used by the Gazebo bridge in
simulation).
"""

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def triangle_angle(t: float, period: float, amplitude: float) -> tuple[float, float]:
    """Return (angle [rad], angular_velocity [rad/s]) at time ``t``.

    A symmetric triangle wave of full period ``period`` and peak ``amplitude``,
    starting at ``-amplitude`` for ``t = 0`` and rising linearly.
    """
    if period <= 0.0:
        return 0.0, 0.0
    phase = (t % period) / period  # in [0, 1)
    if phase < 0.5:
        # rising:  -A  ->  +A
        angle = amplitude * (4.0 * phase - 1.0)
        ang_vel = 4.0 * amplitude / period
    else:
        # falling: +A  ->  -A
        angle = amplitude * (3.0 - 4.0 * phase)
        ang_vel = -4.0 * amplitude / period
    return angle, ang_vel


class LidarSweeper(Node):
    # --- Servo PWM calibration (50 Hz signal) ------------------------------
    # Measured: 4.7 % = -45°, 7.2 % = 0°, 9.7 % = +45°
    # → 2.5 % per 45° → 5.0 % per 90°
    PWM_CENTER = 7.2          # duty cycle (%) at neutral
    PWM_RANGE_PER_90DEG = 5.0  # duty cycle delta per 90 deg

    def __init__(self):
        super().__init__('lidar_sweeper')

        # ----- Parameters --------------------------------------------------
        self.declare_parameter('sim', False)
        self.declare_parameter('amplitude_deg', 30.0)
        # The LD06 publishes scans at 10 Hz.  One full sweep period
        # (down AND back up) is locked to ``scans_per_sweep`` scans, so
        # each half-sweep contains exactly ``scans_per_sweep / 2`` scans
        # at evenly spaced tilt angles.
        self.declare_parameter('lidar_scan_rate_hz', 10.0)
        self.declare_parameter('scans_per_sweep', 20)
        self.declare_parameter('joint_name', 'lidar_joint')
        # Topic merged into /joint_states by joint_state_publisher source_list
        self.declare_parameter('joint_state_topic', '/lidar_joint_states')
        # Servo command topic (used by the Gazebo bridge in sim)
        self.declare_parameter('cmd_topic', '/lidar_cmd_pos')
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('min_angle_deg', -45.0)
        self.declare_parameter('max_angle_deg', 45.0)

        self.sim = bool(self.get_parameter('sim').value)
        self.amplitude = math.radians(float(self.get_parameter('amplitude_deg').value))
        scan_rate = float(self.get_parameter('lidar_scan_rate_hz').value)
        scans_per_sweep = int(self.get_parameter('scans_per_sweep').value)
        self.period = scans_per_sweep / scan_rate  # seconds, full triangle period
        self.joint_name = str(self.get_parameter('joint_name').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        ctrl_rate = float(self.get_parameter('control_rate_hz').value)
        self.angle_min = math.radians(float(self.get_parameter('min_angle_deg').value))
        self.angle_max = math.radians(float(self.get_parameter('max_angle_deg').value))

        # ----- Publishers --------------------------------------------------
        self.cmd_pub = self.create_publisher(Float64, self.cmd_topic, 10)
        self.js_pub = self.create_publisher(JointState, joint_state_topic, 10)

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
                    f'Could not init hardware PWM ({e}); running in command-only mode.'
                )
                self.pwm = None
        else:
            self.get_logger().info('Running in simulation mode - PWM disabled')

        # ----- Sweep epoch -------------------------------------------------
        # Both this node and the assembler evaluate triangle(t % period), where
        # t is absolute ROS time (seconds).  Using absolute time is critical:
        # if this node used a relative epoch (clock.now() - t0) the two nodes
        # would be phase-shifted by (t0 % period), producing wrong tilt angles
        # in the assembler and a vertically-flipped or sheared point cloud.

        self.get_logger().info(
            f'Sweep: ±{math.degrees(self.amplitude):.1f}°, '
            f'period={self.period:.3f}s '
            f'({scans_per_sweep} scans @ {scan_rate:g} Hz, '
            f'half-sweep = {self.period/2:.3f}s).'
        )

        self.timer = self.create_timer(1.0 / ctrl_rate, self.tick)

    # ------------------------------------------------------------------
    def _t_now(self) -> float:
        now = self.get_clock().now()
        return now.nanoseconds * 1e-9

    def tick(self):
        t = self._t_now()
        angle, _vel = triangle_angle(t, self.period, self.amplitude)
        # Hard clamp (in case parameters are misconfigured)
        if angle < self.angle_min:
            angle = self.angle_min
        elif angle > self.angle_max:
            angle = self.angle_max

        stamp = self.get_clock().now().to_msg()

        # 1) Command for Gazebo bridge / logging
        self.cmd_pub.publish(Float64(data=angle))

        # 2) JointState for TF (robot_state_publisher)
        js = JointState()
        js.header.stamp = stamp
        js.name = [self.joint_name]
        js.position = [angle]
        self.js_pub.publish(js)

        # 3) Drive the servo
        if self.pwm is not None:
            duty = self.PWM_CENTER + math.degrees(angle) * self.PWM_RANGE_PER_90DEG / 90.0
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
