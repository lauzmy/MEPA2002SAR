"""Standalone test bringup for the tilted-LD06 -> 3D pointcloud pipeline.

Brings up only what is strictly required to test the sweeper + assembler
on the real robot (or with a recorded /scan bag):

  * robot_state_publisher  (for the laser <-> base_footprint TF)
  * joint_state_publisher  (merges /lidar_joint_states from sweeper)
  * ldlidar driver         (publishes /scan)         -- optional
  * lidar_sweeper          (drives the servo)
  * laser_assembler        (publishes /lidar3d/points)
  * rviz2                  -- optional

Usage:
    ros2 launch ros_gz_bringup lidar3d_test.launch.py
    ros2 launch ros_gz_bringup lidar3d_test.launch.py quality_preset:=fast
    ros2 launch ros_gz_bringup lidar3d_test.launch.py quality_preset:=custom scans_per_sweep:=80
    ros2 launch ros_gz_bringup lidar3d_test.launch.py auto_calibrate:=true
    ros2 launch ros_gz_bringup lidar3d_test.launch.py aggregate_mode:=half_sweep
    ros2 launch ros_gz_bringup lidar3d_test.launch.py lidar:=false   # use bag
    ros2 launch ros_gz_bringup lidar3d_test.launch.py sim:=true       # no PWM
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('ros_gz_description')
    pkg_bringup = get_package_share_directory('ros_gz_bringup')

    # --- Args --------------------------------------------------------------
    sim_arg = DeclareLaunchArgument(
        'sim', default_value='false',
        description='If true, do not drive the hardware PWM (servo).')
    preset_arg = DeclareLaunchArgument(
        'quality_preset', default_value='balanced',
        description='Density/speed trade-off: fast (20), balanced (50), dense (100), custom.')
    scans_arg = DeclareLaunchArgument(
        'scans_per_sweep', default_value='50',
        description='Scans per full sweep. Used when quality_preset=custom.')
    settle_arg = DeclareLaunchArgument(
        'settle_time_s', default_value='0.08',
        description='Time for the servo to physically reach each step. '
                    'Run auto_calibrate:=true to measure this automatically.')
    autocal_arg = DeclareLaunchArgument(
        'auto_calibrate', default_value='false',
        description='Measure settle_time_s from live scans on startup.')
    aggregate_arg = DeclareLaunchArgument(
        'aggregate_mode', default_value='sweep',
        description='Cloud publish cadence: scan | half_sweep | sweep.')
    lidar_arg = DeclareLaunchArgument(
        'lidar', default_value='true',
        description='Start the ldlidar driver. Set false when replaying a bag.')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Open RViz2 with a minimal config.')
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyAMA0',
        description='Serial port for the LD06.')

    sim = LaunchConfiguration('sim')
    use_lidar = LaunchConfiguration('lidar')

    # --- Robot description (xacro -> URDF) --------------------------------
    xacro_file = os.path.join(
        pkg_description, 'models', 'xacro_test', 'test.urdf.xacro')
    robot_desc = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_desc,
        }],
    )

    # joint_state_publisher merges the tilt-joint state coming from
    # lidar_sweeper into /joint_states so the laser TF actually moves.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'source_list': ['/lidar_joint_states'],
            'rate': 100,
        }],
    )

    # --- LD06 driver (optional) -------------------------------------------
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyAMA0',
            'topic_name': '/scan',
            'lidar_frame': 'laser',
            'lidar_type': 'LD06',
        }],
        condition=IfCondition(use_lidar),
    )

    # --- Sweeper + assembler ---------------------------------------------
    lidar_sweeper = Node(
        package='ros_gz_application',
        executable='lidar_sweeper',
        name='lidar_sweeper',
        output='screen',
        parameters=[{
            'sim': sim,
            'min_angle_deg': -30.0,
            'max_angle_deg': 30.0,
            'quality_preset': LaunchConfiguration('quality_preset'),
            'scans_per_sweep': LaunchConfiguration('scans_per_sweep'),
            'settle_time_s': LaunchConfiguration('settle_time_s'),
            # hold_time_s left unset (<=0) so it auto-derives from scan rate
            'lidar_scan_rate_hz': 10.0,
            'hold_margin_s': 0.02,
            'joint_name': 'lidar_joint',
            'joint_state_topic': '/lidar_joint_states',
            'joint_state_rate_hz': 100.0,
            'hold_state_topic': '/lidar_hold_state',
            'cmd_topic': '/lidar_cmd_pos',
            'scan_topic': '/scan',
            'output_frame': 'base_footprint',
            'auto_calibrate_on_start': LaunchConfiguration('auto_calibrate'),
        }],
    )

    laser_assembler = Node(
        package='ros_gz_application',
        executable='laser_assembler',
        name='tilt_laser_assembler',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'scan_topic': '/scan',
            'cloud_topic': '/lidar3d/points',
            'hold_state_topic': '/lidar_hold_state',
            'joint_state_topic': '/lidar_joint_states',
            'joint_name': 'lidar_joint',
            'output_frame': 'base_footprint',
            'settle_guard_s': 0.02,
            'aggregate_mode': LaunchConfiguration('aggregate_mode'),
            'scans_per_sweep': LaunchConfiguration('scans_per_sweep'),
        }],
    )

    # --- RViz (optional) --------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'test_robot.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        sim_arg,
        lidar_arg,
        rviz_arg,
        serial_port_arg,
        preset_arg,
        scans_arg,
        settle_arg,
        autocal_arg,
        aggregate_arg,
        robot_state_publisher,
        joint_state_publisher,
        ldlidar_node,
        lidar_sweeper,
        laser_assembler,
        rviz,
    ])
