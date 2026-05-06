"""Run the LD06 driver + servo settle-time calibrator.

Brings up only what's strictly needed to measure how long the tilt
servo takes to physically reach a commanded angle:

  * ldlidar driver  -> /scan
  * lidar_settle_calibrator (drives the PWM directly, reads /scan,
                             prints a recommended settle_time_s)

Do NOT run lidar_sweeper at the same time -- the calibrator owns the
hardware PWM channel.

Usage:
    ros2 launch ros_gz_bringup lidar_calibrate.launch.py
    ros2 launch ros_gz_bringup lidar_calibrate.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyAMA0',
        description='Serial port for the LD06.')
    home_arg = DeclareLaunchArgument(
        'home_deg', default_value='0.0',
        description='Reference (home) tilt angle in degrees.')

    ldlidar = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'topic_name': '/scan',
            'lidar_frame': 'laser',
            'lidar_type': 'LD06',
        }],
    )

    calibrator = Node(
        package='ros_gz_application',
        executable='lidar_settle_calibrator',
        name='lidar_settle_calibrator',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'home_deg': LaunchConfiguration('home_deg'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        home_arg,
        ldlidar,
        calibrator,
    ])
