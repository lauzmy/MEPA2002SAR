"""3D mapping bringup — real robot and Gazebo simulation.

Switch between modes by editing the top of config/IRL/3d_mapping.yaml:

    sim: false   →  real robot  (ldlidar + allocator + UART servo)
    sim: true    →  Gazebo sim  (gz_sim + ros_gz_bridge + spawn)

All other node parameters live in that same config file.

Usage:
    ros2 launch ros_gz_bringup 3dMapping.launch.py
"""

import os
import tempfile
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description():
    pkg_description = get_package_share_directory('gregor_description')
    pkg_bringup     = get_package_share_directory('ros_gz_bringup')

    config_file = os.path.join(pkg_bringup, 'config', 'IRL', '3d_mapping.yaml')
    urdf_file   = os.path.join(pkg_description, 'urdf', 'gregor.urdf')

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    use_sim = bool(config.get('_launch_settings', {}).get('ros__parameters', {}).get('sim', False))

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # -----------------------------------------------------------------------
    # robot_state_publisher — always needed
    # -----------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim,
            'robot_description': robot_desc,
        }],
    )

    nodes = [robot_state_publisher]

    # -----------------------------------------------------------------------
    # Simulation mode
    # -----------------------------------------------------------------------
    if use_sim:

        discovery_range = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
        nodes.insert(0, discovery_range)


        pkg_ros_gz_sim     = get_package_share_directory('ros_gz_sim')
        pkg_example_gazebo = get_package_share_directory('ros_gz_gazebo')
        pkg_ros_gz_desc    = get_package_share_directory('ros_gz_description')

        # Process world xacro → temp SDF (reuse existing test_world arena).
        # Note: the world already includes test_robot — gregor is spawned
        # separately below and will appear alongside it.
        world_xacro = os.path.join(pkg_example_gazebo, 'worlds', 'test_xacro.sdf.xacro')
        world_sdf   = subprocess.check_output(['xacro', world_xacro]).decode('utf-8')
        with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as wf:
            wf.write(world_sdf)
            world_file_path = wf.name

        # Set env vars directly in Python so all child processes (including
        # Gazebo) inherit them at spawn time. SetEnvironmentVariable alone is
        # not reliable for IncludeLaunchDescription subprocesses.

        # GZ_SIM_RESOURCE_PATH: parent of gregor_description share dir so
        # Gazebo resolves model://gregor_description/meshes/... URIs.
        _gz_path_parts = [
            os.path.join(pkg_ros_gz_desc, 'models'),
            os.path.dirname(pkg_description),
        ]
        if os.environ.get('GZ_SIM_RESOURCE_PATH', ''):
            _gz_path_parts.append(os.environ['GZ_SIM_RESOURCE_PATH'])
        os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(_gz_path_parts)

        # Mirror as launch actions for introspection.
        gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.environ['GZ_SIM_RESOURCE_PATH'])

        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
        )

        # Bridge: connects Gazebo topics ↔ ROS topics.
        # See config/sim/3d_mapping_bridge.yaml for the full mapping.
        ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{
                'config_file': os.path.join(
                    pkg_bringup, 'config', 'sim', '3d_mapping_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
        )

        # Spawn gregor from the robot_description topic that
        # robot_state_publisher advertises.
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_gregor',
            arguments=[
                '-name', 'gregor',
                '-topic', 'robot_description',
                '-x', '2.0', '-y', '0.0', '-z', '1.0',
            ],
            output='screen',
        )

        # lidar3d in sim mode: no UART, no PWM — uses commanded angle only.
        # Gazebo's JointStatePublisher bridge covers /joint_states (all joints
        # including the tilt), so no ROS joint_state_publisher is needed here.
        lidar3d = Node(
            package='ros_gz_application',
            executable='lidar3d',
            name='lidar3d',
            output='screen',
            parameters=[config_file, {
                'sim': True,
                'use_sim_time': True,
            }],
        )

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
            parameters=[{'use_sim_time': True}],
        )

        nodes += [gz_resource_path,
                  gz_sim, ros_gz_bridge, spawn, lidar3d, rviz]

    # -----------------------------------------------------------------------
    # Real-robot mode
    # -----------------------------------------------------------------------
    else:
        # joint_state_publisher merges the UART-measured tilt angle from
        # lidar3d into /joint_states for robot_state_publisher.
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

        ldlidar = Node(
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar',
            output='screen',
            parameters=[config_file],
        )

        lidar3d = Node(
            package='ros_gz_application',
            executable='lidar3d',
            name='lidar3d',
            output='screen',
            parameters=[config_file],
        )

        allocator = Node(
            package='ros_gz_application',
            executable='allocator',
            name='allocator',
            output='screen',
            parameters=[config_file],
        )

        nodes += [joint_state_publisher, ldlidar, lidar3d, allocator]

    return LaunchDescription(nodes)
