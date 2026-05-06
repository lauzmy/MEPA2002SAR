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
    ros2 launch ros_gz_bringup lidar3d_test.launch.py rviz:=true
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
            'rate': 50,
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
    # Keep the sweep parameters in lockstep between the two nodes!
    sweep_params = {
        'amplitude_deg': 30.0,
        'lidar_scan_rate_hz': 10.0,   # LD06 = 10 rev/s
        'scans_per_sweep': 40,        # full down+up = 2.0 s
    }

    lidar_sweeper = Node(
        package='ros_gz_application',
        executable='lidar_sweeper',
        name='lidar_sweeper',
        output='screen',
        parameters=[{
            **sweep_params,
            'sim': False,                 # set true to skip PWM writes (for testing without servo)
            'joint_name': 'lidar_joint',
            'joint_state_topic': '/lidar_joint_states',
            'cmd_topic': '/lidar_cmd_pos',
            'control_rate_hz': 100.0,
        }],
    )

    laser_assembler = Node(
        package='ros_gz_application',
        executable='laser_assembler',
        name='tilt_laser_assembler',
        output='screen',
        parameters=[{
            **sweep_params,
            'use_sim_time': False,
            'servo_lag_s': 0.0,          # tune if cloud "shears" along sweep dir
            'pivot_offset_m': 0.0325,     # 32.5 mm laser plane above pivot
            'mount_xyz': [0.0, 0.0, 0.15],
            'scan_topic': '/scan',
            'cloud_topic': '/lidar3d/points',
            'output_frame': 'base_footprint',
            'beam_angle_sign': 1.0,       # set -1.0 if cloud is mirrored
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
        robot_state_publisher,
        joint_state_publisher,
        ldlidar_node,
        lidar_sweeper,
        laser_assembler,
        rviz,
    ])
