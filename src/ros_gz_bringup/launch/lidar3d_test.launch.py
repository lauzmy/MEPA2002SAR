"""Standalone test bringup for the tilted-LD06 -> 3D pointcloud pipeline.

Brings up only what is strictly required to test the unified lidar3d
node on the real robot (or with a recorded /scan bag):

  * robot_state_publisher  (for the laser <-> body_link TF)
  * joint_state_publisher  (merges /lidar_joint_states from lidar3d)
  * ldlidar driver         (publishes /scan)         -- optional
  * lidar3d                (sweeps the servo, reads UART, publishes
                            /lidar3d/points)
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
    sweep_period_arg = DeclareLaunchArgument(
        'sweep_period_s', default_value='4.0',
        description='Period of the continuous tilt triangle sweep.')
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
    # lidar3d into /joint_states so the laser TF actually moves.
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

    # --- Unified 3D lidar node -------------------------------------------
    lidar3d = Node(
        package='ros_gz_application',
        executable='lidar3d',
        name='lidar3d',
        output='screen',
        parameters=[{
            'sim': False,
            'min_angle_deg': -30.0,
            'max_angle_deg': 30.0,
            'sweep_period_s': LaunchConfiguration('sweep_period_s'),
            'scan_topic': '/scan',
            'cloud_topic': '/lidar3d/points',
            'joint_state_topic': '/lidar_joint_states',
            'cmd_topic': '/lidar_cmd_pos',
            'joint_name': 'lidar_joint',
            'output_frame': 'body_link',
            'uart_port': '/dev/ttyAMA1',
            'uart_baud': 921600,
            'pwm_chip': 0,
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
        sweep_period_arg,
        robot_state_publisher,
        joint_state_publisher,
        ldlidar_node,
        lidar3d,
        rviz,
    ])
