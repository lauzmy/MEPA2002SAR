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
from datetime import datetime

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


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description():
    pkg_description = get_package_share_directory('gregor_description')
    pkg_bringup     = get_package_share_directory('ros_gz_bringup')

    # Allow the user to skip live MOLA-LO entirely (e.g. when only
    # recording a bag for offline `mola-map.sh` processing). The MOLA
    # node tree is otherwise included in both sim and real branches.
    enable_mola_arg = DeclareLaunchArgument(
        'enable_mola', default_value='true',
        description='If true, include mola_lo.launch.py for live SLAM. '
                    'Set to false to bring up only the sensors / drivers / '
                    'EKF and record a bag for offline processing.')
    enable_mola = LaunchConfiguration('enable_mola')

    # Bag recording is on by default in sim (kept inside the sim branch
    # for backwards compatibility) and exposed as a launch arg on real
    # hardware so a teleop-only debug session can skip the recorder.
    record_bag_arg = DeclareLaunchArgument(
        'record_bag', default_value='false',
        description='If true (real-robot mode), record a rosbag with all '
                    'topics needed for offline mola-map.sh.')
    record_bag = LaunchConfiguration('record_bag')

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
        # Z = 0.05 keeps the wheels just above the ground plane so
        # physics settles in <1 frame. Spawning at 1.0 m caused the
        # chassis to drop and bounce, and the first /lidar3d/points
        # cloud (which MOLA uses to define the `map` frame) was being
        # captured mid-tilt, locking the whole map at an offset angle.
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_gregor',
            arguments=[
                '-name', 'gregor',
                '-topic', 'robot_description',
                '-x', '2.0', '-y', '0.0', '-z', '0.05',
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

        # robot_localization EKF: fuses Gazebo wheel odometry (planar twist)
        # and IMU (yaw rate) into /odometry/filtered, and owns the
        # odom→base_footprint TF (the MecanumDrive plugin's tf is no longer
        # bridged — see config/sim/3d_mapping_bridge.yaml). MOLA's offline
        # processing reads /odometry/filtered as its motion prior.
        ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(pkg_bringup, 'config', 'sim', 'ekf.yaml'),
                {'use_sim_time': True},
            ],
        )

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
            parameters=[{'use_sim_time': True}],
        )

        # Bag recorder — captures all data needed for offline MOLA processing:
        #   /lidar3d/points  →  3D LiDAR point cloud (primary sensor)
        #   /wheel/odometry  →  encoder-based odometry
        #   /tf + /tf_static →  sensor pose on vehicle
        bag_output = os.path.join(
            os.path.expanduser('~'),
            'ros2_ws',
            'rosbags',
            datetime.now().strftime('3d_mapping_%Y%m%d_%H%M%S'),
        )
        rosbag_record = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                # robot_state_publisher publishes /tf_static with
                # transient_local durability; without --include-hidden-topics
                # rosbag2's late subscription can miss it, leaving offline
                # MOLA without base_footprint↔base_link et al.
                '--include-hidden-topics',
                '-o', bag_output,
                '/lidar3d/points',
                '/wheel/odometry',      # raw, for diagnostics / re-fusion
                '/odometry/filtered',   # EKF output — MOLA's motion prior
                '/imu',
                '/tf',
                '/tf_static',
            ],
            output='screen',
        )

        nodes += [gz_resource_path,
                  gz_sim, ros_gz_bridge, spawn, lidar3d, ekf_node, rviz, rosbag_record]

        # MOLA-LO live: same pipeline / state estimator as the offline
        # mola-map.sh script, fed by /lidar3d/points + /odometry/filtered.
        # Publishes map→odom on /tf and the live aggregated cloud on
        # /mola_lo/local_map_points.
        mola_lo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_bringup, 'launch', 'mola_lo.launch.py')),
            launch_arguments={
                'use_sim_time':    'true',
                'lidar_topic':     '/lidar3d/points',
                'odom_topic':      '/odometry/filtered',
                'imu_topic':       '/imu',
                # MOLA's REP-105 base. tf2 resolves `odom -> base_link`
                # through the chain `odom -> base_footprint` (EKF) +
                # `base_footprint -> base_link` (RSP). We previously
                # tried `base_footprint` here to avoid a 2-hop chain,
                # but that caused MOLA's built-in static
                # base_footprint -> base_link_frame broadcast to become
                # base_footprint -> base_footprint (self-TF spam). The
                # env var MOLA_TF_FOOTPRINT_LINK='' in mola_lo.launch.py
                # disables that broadcast entirely, leaving RSP as the
                # single owner of base_footprint -> base_link.
                'base_link_frame': 'base_link',
            }.items(),
            condition=IfCondition(enable_mola),
        )
        # Delay MOLA-LO startup so that the EKF (robot_localization) has
        # had time to publish its first odom -> base_footprint TF before
        # MOLA's BridgeROS2 tries the REP-105 lookup. Without the delay,
        # MOLA's first publish_localization_following_rep105 fires
        # before /tf has `odom`, producing a one-shot
        #   "odom passed to lookupTransform does not exist" error,
        # plus three TF_NO_*/TF_SELF_TRANSFORM errors from the empty
        # placeholder transform that BridgeROS2 emits in that window.
        #
        # 8 s (was 3 s): with `forward_ros_tf_odom_to_mola=True` and
        # `imu_gravity_correction=True`, BridgeROS2 starts looking up
        # /tf earlier in its init, so the previous 3 s window was
        # occasionally insufficient on cold starts where Gazebo took
        # 4-6 s to spawn the model + start ticking /clock + let the EKF
        # converge enough to publish its first odom -> base_footprint.
        # 8 s comfortably covers that on a desktop; bump further if you
        # see the empty-frame errors persist past launch.
        nodes.append(TimerAction(period=8.0, actions=[mola_lo]))

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

        # BNO085 IMU on the I²C-4 bus. Publishes /imu/data at 50 Hz with
        # frame_id="imu_link" (matches the URDF static TF broadcast by
        # robot_state_publisher), so MOLA's gravity correction can rotate
        # the gravity vector into base_link correctly. Mirrors the sim
        # branch's reliance on Gazebo's IMU sensor + <gz_frame_id>imu_link
        # </gz_frame_id> tag in gregor.urdf.
        imu_node = Node(
            package='ros_gz_application',
            executable='IMU',
            name='imu_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'i2c_bus': 4,
            }],
        )

        # robot_localization EKF: fuses /wheel/odometry (planar twist from
        # the allocator's encoder integration) and /imu/data (yaw rate
        # from the BNO085) into /odometry/filtered, and owns the
        # odom→base_footprint TF. MOLA-LO consumes /odometry/filtered as
        # its motion prior \u2014 same architecture as the sim branch, so
        # tuning of state-estimation-simple.yaml carries over 1:1.
        ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(pkg_bringup, 'config', 'IRL', 'ekf_imu.yaml'),
                {'use_sim_time': False},
            ],
        )

        nodes += [joint_state_publisher, ldlidar, lidar3d, allocator,
                  imu_node, ekf_node]

        # Bag recorder (real robot). Captures everything the offline
        # `scripts/mola-map.sh` needs to rebuild the map after the run:
        #   /lidar3d/points  — 3D cloud assembled by lidar3d
        #   /wheel/odometry  — raw mecanum odometry from allocator
        #   /odometry/filtered — EKF output (MOLA's motion prior)
        #   /imu/data        — BNO085 (50 Hz, frame_id=imu_link)
        #   /tf + /tf_static — sensor poses (RSP + JSP + EKF)
        # `--include-hidden-topics` is required so /tf_static (transient
        # local) is captured; without it rosbag2's late subscription
        # routinely misses it.
        bag_output = os.path.join(
            os.path.expanduser('~'),
            'ros2_ws',
            'rosbags',
            datetime.now().strftime('3d_mapping_%Y%m%d_%H%M%S'),
        )
        rosbag_record = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--include-hidden-topics',
                '-o', bag_output,
                '/lidar3d/points',
                '/wheel/odometry',
                '/odometry/filtered',
                '/imu/data',
                '/tf',
                '/tf_static',
            ],
            output='screen',
            condition=IfCondition(record_bag),
        )
        nodes.append(rosbag_record)

        # MOLA-LO live (real robot). Now mirrors the sim branch: feed the
        # EKF-fused /odometry/filtered as motion prior, and the BNO085's
        # /imu/data as gravity source for ICP roll/pitch correction.
        mola_lo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_bringup, 'launch', 'mola_lo.launch.py')),
            launch_arguments={
                'use_sim_time':    'false',
                'lidar_topic':     '/lidar3d/points',
                'odom_topic':      '/odometry/filtered',
                'imu_topic':       '/imu/data',
                # See sim branch comment + MOLA_TF_FOOTPRINT_LINK='' env
                # var in mola_lo.launch.py. tf2 resolves the chain via
                # the EKF (odom -> base_footprint) + RSP (base_footprint
                # -> base_link).
                'base_link_frame': 'base_link',
            }.items(),
            condition=IfCondition(enable_mola),
        )
        # Same rationale as the sim branch: give RSP / joint_state_publisher
        # / lidar3d a head start so /tf is fully populated before MOLA's
        # first REP-105 lookup. There's no EKF on the real robot, so the
        # critical chain is base_footprint -> base_link (RSP) plus the
        # first /lidar3d/points cloud arriving. 8 s mirrors the sim
        # branch's expanded window (was 3 s) — see comment there.
        nodes.append(TimerAction(period=8.0, actions=[mola_lo]))

    return LaunchDescription([enable_mola_arg, record_bag_arg] + nodes)
