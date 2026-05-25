"""3D mapping bringup for real robot and Gazebo sim.

Switch modes by editing config/IRL/3d_mapping.yaml:
    sim: false   →  real robot  (ldlidar + allocator + UART servo)
    sim: true    →  Gazebo sim  (gz_sim + ros_gz_bridge + spawn)

All other node parameters live in that same config file.

Usage:
    ros2 launch gregor_bringup 3dMapping.launch.py
"""

# --- Imports ---
# stdlib
import os
import subprocess
import tempfile
from datetime import datetime

# third-party
import yaml

# ROS
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Delay MOLA-LO startup so RSP/EKF/Gazebo populate /tf before MOLA's first REP-105
# lookup. See wiki: Bringup/MolaLo-Timing.
MOLA_LO_STARTUP_DELAY_S = 8.0

# Bag recorder captures the topics offline mola-map.sh needs. /tf_static is
# transient_local — --include-hidden-topics is required or rosbag2's late
# subscription misses it.
COMMON_BAG_TOPICS = [
    '/lidar3d/points',
    '/wheel/odometry',
    '/odometry/filtered',
    '/tf',
    '/tf_static',
]


def generate_launch_description():
    pkg_description = get_package_share_directory('gregor_description')
    pkg_bringup     = get_package_share_directory('gregor_bringup')

    # --- Launch arguments ---
    enable_mola_arg = DeclareLaunchArgument(
        'enable_mola', default_value='true',
        description='Include mola_lo.launch.py for live SLAM. Set false to record bags for offline mola-map.sh.')
    enable_mola = LaunchConfiguration('enable_mola')

    record_bag_arg = DeclareLaunchArgument(
        'record_bag', default_value='false',
        description='Real-robot only: record a rosbag for offline mola-map.sh.')
    record_bag = LaunchConfiguration('record_bag')

    config_file = os.path.join(pkg_bringup, 'config', 'IRL', '3d_mapping.yaml')
    urdf_file   = os.path.join(pkg_description, 'urdf', 'gregor.urdf')

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    use_sim = bool(config.get('_launch_settings', {}).get('ros__parameters', {}).get('sim', False))

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # --- robot_state_publisher (always needed) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim, 'robot_description': robot_desc}],
    )

    nodes = [robot_state_publisher]

    if use_sim:
        nodes += _sim_branch(pkg_bringup, pkg_description, config_file, enable_mola)
    else:
        nodes += _real_branch(pkg_bringup, config_file, enable_mola, record_bag)

    return LaunchDescription([enable_mola_arg, record_bag_arg] + nodes)


# --- Simulation branch ---

def _sim_branch(pkg_bringup, pkg_description, config_file, enable_mola):
    pkg_ros_gz_sim     = get_package_share_directory('ros_gz_sim')
    pkg_example_gazebo = get_package_share_directory('gregor_sim_description')
    pkg_ros_gz_desc    = get_package_share_directory('gregor_sim_description')

    discovery_range = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    # World xacro → temp SDF (reuses the test_world arena; gregor is spawned separately).
    world_xacro = os.path.join(pkg_example_gazebo, 'worlds', 'test_xacro.sdf.xacro')
    world_sdf   = subprocess.check_output(['xacro', world_xacro]).decode('utf-8')
    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as wf:
        wf.write(world_sdf)
        world_file_path = wf.name

    # Set GZ_SIM_RESOURCE_PATH directly in os.environ — SetEnvironmentVariable alone
    # doesn't reliably propagate to IncludeLaunchDescription subprocesses.
    _gz_path_parts = [
        os.path.join(pkg_ros_gz_desc, 'models'),
        os.path.dirname(pkg_description),
    ]
    if os.environ.get('GZ_SIM_RESOURCE_PATH', ''):
        _gz_path_parts.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(_gz_path_parts)
    gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.environ['GZ_SIM_RESOURCE_PATH'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
    )

    # Bridge between Gazebo and ROS topics; mapping lives in config/sim/3d_mapping_bridge.yaml.
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_bringup, 'config', 'sim', '3d_mapping_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
    )

    # Z=0.05 keeps wheels just above ground so physics settles in <1 frame. See wiki: Bringup/SimSpawn.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_gregor',
        arguments=['-name', 'gregor', '-topic', 'robot_description', '-x', '2.0', '-y', '0.0', '-z', '0.05'],
        output='screen',
    )

    # lidar3d in sim: commanded angle only (no UART/PWM). Gazebo's bridge feeds /joint_states.
    lidar3d = Node(
        package='gregor_application',
        executable='lidar3d',
        name='lidar3d',
        output='screen',
        parameters=[config_file, {'sim': True, 'use_sim_time': True}],
    )

    # EKF fuses Gazebo wheel odom + IMU into /odometry/filtered and owns odom→base_footprint TF.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_bringup, 'config', 'sim', 'ekf.yaml'), {'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_bringup, 'config', '3d_mapping.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    bag_output = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'rosbags',
        datetime.now().strftime('3d_mapping_%Y%m%d_%H%M%S'),
    )
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--include-hidden-topics', '-o', bag_output,
             *COMMON_BAG_TOPICS, '/imu'],
        output='screen',
    )

    mola_lo = _make_mola_include(pkg_bringup, sim=True, imu_topic='/imu', enable_mola=enable_mola)

    return [
        discovery_range,
        gz_resource_path,
        gz_sim, ros_gz_bridge, spawn, lidar3d, ekf_node, rviz, rosbag_record,
        TimerAction(period=MOLA_LO_STARTUP_DELAY_S, actions=[mola_lo]),
    ]


# --- Real-robot branch ---

def _real_branch(pkg_bringup, config_file, enable_mola, record_bag):
    # Merges UART-measured tilt from lidar3d into /joint_states for robot_state_publisher.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False, 'source_list': ['/lidar_joint_states'], 'rate': 50}],
    )

    ldlidar = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[config_file],
    )

    lidar3d = Node(
        package='gregor_application',
        executable='lidar3d',
        name='lidar3d',
        output='screen',
        parameters=[config_file],
    )

    allocator = Node(
        package='gregor_application',
        executable='allocator',
        name='allocator',
        output='screen',
        parameters=[config_file],
    )

    # BNO085 on I²C-4 publishing /imu/data at 50 Hz with frame_id=imu_link.
    imu_node = Node(
        package='gregor_application',
        executable='IMU',
        name='imu_node',
        output='screen',
        parameters=[{'use_sim_time': False, 'i2c_bus': 4}],
    )

    # EKF fuses /wheel/odometry (allocator) + /imu/data (BNO085) into /odometry/filtered.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_bringup, 'config', 'IRL', 'ekf_imu.yaml'), {'use_sim_time': False}],
    )

    bag_output = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'rosbags',
        datetime.now().strftime('3d_mapping_%Y%m%d_%H%M%S'),
    )
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--include-hidden-topics', '-o', bag_output,
             *COMMON_BAG_TOPICS, '/imu/data'],
        output='screen',
        condition=IfCondition(record_bag),
    )

    mola_lo = _make_mola_include(pkg_bringup, sim=False, imu_topic='/imu/data', enable_mola=enable_mola)

    return [
        joint_state_publisher, ldlidar, lidar3d, allocator, imu_node, ekf_node,
        rosbag_record,
        TimerAction(period=MOLA_LO_STARTUP_DELAY_S, actions=[mola_lo]),
    ]


def _make_mola_include(pkg_bringup, *, sim: bool, imu_topic: str, enable_mola):
    # base_link_frame=base_link relies on the EKF/RSP chain odom→base_footprint→base_link.
    # MOLA_TF_FOOTPRINT_LINK='' in mola_lo.launch.py disables MOLA's self-broadcast.
    # See wiki: Bringup/MolaLo-Frames.
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'mola_lo.launch.py')),
        launch_arguments={
            'use_sim_time':    'true' if sim else 'false',
            'lidar_topic':     '/lidar3d/points',
            'odom_topic':      '/odometry/filtered',
            'imu_topic':       imu_topic,
            'base_link_frame': 'base_link',
        }.items(),
        condition=IfCondition(enable_mola),
    )
