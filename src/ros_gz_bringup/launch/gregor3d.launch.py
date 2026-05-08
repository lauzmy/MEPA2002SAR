"""Gregor 3D-SLAM bringup (MOLA LIO + tilted LD06 + IMU + EKF).

Pipeline:
    /scan (LD06)                              Per-revolution PointCloud2
        \\                                      (10 Hz, per-point 'time'
         +-> lidar3d (tilt sweep + UART) -->     field, frame=body_link)
        /                                                |
    UART (servo angle, 50+ Hz)                           v
                                            mola_lidar_odometry (LIO)
    /imu/data (BNO085, 50 Hz) -----------------------> map -> odom TF
                                                         (REP-105)

    /wheel/odometry + /imu/data --> ekf_node --> odom -> base_footprint

Frames (URDF):
    base_footprint -> body_link -> { imu_link, lidar_joint -> laser, ... }
    NOTE: there is *no* base_link, so MOLA's `mola_tf_base_link` must be
    set to base_footprint (matches the EKF child frame).

Optional 2D projection for Nav2 + explore_lite (`octomap:=true`):
    /lidar3d/points -> octomap_server -> /projected_map (-> /map)

Usage:
    ros2 launch ros_gz_bringup gregor3d.launch.py
    ros2 launch ros_gz_bringup gregor3d.launch.py rviz:=true
    ros2 launch ros_gz_bringup gregor3d.launch.py nav2:=true octomap:=true \\
        explore:=true
    ros2 launch ros_gz_bringup gregor3d.launch.py \\
        mola_output_dir:=$HOME/maps/lab_run_01
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ros_gz_bringup')
    pkg_description = get_package_share_directory('ros_gz_description')

    # -------------------------------------------------------------------- args
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                     description='Open RViz2.')
    slam_arg = DeclareLaunchArgument('slam', default_value='true',
                                     description='Run MOLA 3D LIO.')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='false',
                                     description='Run Nav2 (also enable '
                                     'octomap to feed it a 2D /map).')
    octomap_arg = DeclareLaunchArgument(
        'octomap', default_value='false',
        description='Run octomap_server to project /lidar3d/points into a '
        '2D OccupancyGrid (/projected_map -> /map) so Nav2 + explore_lite '
        'keep working unchanged.')
    explore_arg = DeclareLaunchArgument('explore', default_value='false',
                                        description='Run explore_lite '
                                        '(needs octomap:=true and nav2:=true).')
    sweep_period_arg = DeclareLaunchArgument(
        'sweep_period_s', default_value='4.0',
        description='Servo tilt sweep period for lidar3d (cloud rate is '
        'still 10 Hz; this only sets how fast the tilt scans up/down).')
    mola_warmup_arg = DeclareLaunchArgument(
        'mola_warmup_s', default_value='6.0',
        description='Seconds to wait (robot stationary) before starting '
        'MOLA so the first frame defines a clean map origin.')
    mola_output_dir_arg = DeclareLaunchArgument(
        'mola_output_dir',
        default_value=os.path.expanduser('~/mola_maps/latest'),
        description='Where MOLA writes its simplemap / state at shutdown.')

    # ------------------------------------------------------- robot description
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

    # joint_state_publisher merges the lidar tilt joint into /joint_states
    # so the laser TF actually moves (lidar3d publishes /lidar_joint_states
    # with the *measured* tilt from the UART).
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

    # ------------------------------------------------------- robot HW + nodes
    allocator_node = Node(
        package='ros_gz_application',
        executable='allocator',
        name='allocator_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # NOTE: BNO085 is mounted with rpy="0 0 0" w.r.t. body_link in the
    # URDF.  The chip's local axes must therefore physically match
    # REP-103 (X forward, Y left, Z up).  If the map ends up tilted,
    # rotate `imu_link` in test.urdf.xacro -- do NOT swap axes inside
    # IMU.py.
    IMU_node = Node(
        package='ros_gz_application',
        executable='IMU',
        name='imu_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'i2c_bus': 4,
        }],
    )

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
    )

    thermal_Reading = Node(
        package='ros_gz_application',
        executable='thermal_reading',
        name='thermal_Reading',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    thermal_processor = Node(
        package='ros_gz_application',
        executable='thermal_processing',
        name='thermal_processor',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # --- Wheel + IMU -> odom EKF (planar, owns odom -> base_footprint) ----
    # two_d_mode stays True: MOLA owns the full 3D pose in `map`, EKF only
    # needs to give a smooth planar odom for the wheels.
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

    # --- Tilted LD06 -> 3D PointCloud2 (10 Hz, body_link frame) ----------
    # One PointCloud2 per LD06 revolution; each point already carries the
    # measured tilt baked in plus a `time` field for IMU deskew.
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
            'angle_invert': True,
            'pwm_chip': 1,   # Pi 5: pwmchip1 is the usable RP1 PWM
        }],
    )

    # ------------------------------------------------------------ MOLA LIO
    # Map persistence: tell MOLA where to dump its state at shutdown.
    # These env vars are read by the upstream launch file.
    mola_env_simplemap = SetEnvironmentVariable(
        name='MOLA_SIMPLEMAP_OUTPUT',
        value=[LaunchConfiguration('mola_output_dir'), '/simplemap.simplemap'],
    )
    mola_env_generate = SetEnvironmentVariable(
        name='MOLA_GENERATE_SIMPLEMAP',
        value='true',
    )
    mola_env_state = SetEnvironmentVariable(
        name='MOLA_STATE_DIR',
        value=LaunchConfiguration('mola_output_dir'),
    )

    pkg_mola_lo = get_package_share_directory('mola_lidar_odometry')
    mola_lio_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mola_lo, 'ros2-launchs', 'ros2-lidar-odometry.launch.py')
        ),
        launch_arguments={
            'lidar_topic_name': '/lidar3d/points',
            'imu_topic_name': '/imu/data',
            # URDF chain is base_footprint -> base_link -> body_link -> ...
            # MOLA's launcher always broadcasts a static TF
            #   base_footprint -> <mola_tf_base_link>
            # so these two MUST be different names, otherwise we get the
            # 'TF_SELF_TRANSFORM ... base_footprint' spam and a broken tree.
            'mola_tf_base_link': 'base_link',
            'use_imu_for_lio': 'True',
            # Let robot_localization keep ownership of odom->base_footprint;
            # MOLA only adds map->odom (REP-105 compliant).
            'forward_ros_tf_odom_to_mola': 'True',
            'publish_localization_following_rep105': 'True',
            # MolaViz (nanogui/OpenGL) segfaults on the Pi 5's aarch64 GL
            # stack ("VertexArrayObject ... m_state.get().created"), and
            # because the upstream launch wraps mola-cli with
            # on_exit=Shutdown() that crash brings the whole bringup down
            # (lidar3d/allocator/octomap/rviz/explore all die with it).
            # We don't need MolaViz anyway -- the live cloud + map are
            # already visualised in our own RViz config.
            'use_mola_gui': 'False',
            # We launch our own RViz (gregor3d.rviz); skip the duplicate
            # one bundled with mola_lidar_odometry to save resources and
            # avoid two RViz instances fighting for GL on the Pi.
            'use_rviz': 'False',
        }.items(),
    )

    # Delay MOLA so the robot is stationary when the first frame is taken
    # (that frame defines the map origin).  Also gives EKF + IMU + UART
    # a moment to start streaming.
    mola_lio = TimerAction(
        period=LaunchConfiguration('mola_warmup_s'),
        actions=[
            LogInfo(msg=':: starting MOLA LIO -- keep the robot still ::'),
            mola_env_simplemap, mola_env_generate, mola_env_state,
            mola_lio_include,
        ],
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    # ------------------------------------------------------------- octomap
    # Projects /lidar3d/points into a 2D OccupancyGrid for Nav2/explore.
    # Apt: ros-${ROS_DISTRO}-octomap-server2 (or octomap-server).
    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'map',
            'base_frame_id': 'base_footprint',
            'resolution': 0.08,
            'sensor_model.max_range': 8.0,
            # Project everything between these heights down to the 2D map.
            'occupancy_min_z': 0.05,
            'occupancy_max_z': 1.50,
            'filter_ground': False,
            'publish_2d_map': True,
        }],
        remappings=[
            ('cloud_in', '/lidar3d/points'),
            # Make the 2D projection available as /map for Nav2/explore.
            ('projected_map', '/map'),
        ],
        condition=IfCondition(LaunchConfiguration('octomap')),
    )

    # ----------------------------------------------------- optional Nav2
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(
        pkg_bringup, 'config', 'IRL', 'nav2_params_smac2D.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params_file,
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    # ----------------------------------------------- optional explore_lite
    explore_lite_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'sim', 'explore.yaml'),
            {'use_sim_time': False},
        ],
    )
    explore = TimerAction(
        period=18.0,   # after Nav2 + MOLA + octomap have warmed up
        actions=[explore_lite_node],
        condition=IfCondition(LaunchConfiguration('explore')),
    )

    # ---------------------------------------------------------------- RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'gregor3d.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    ready_message = TimerAction(
        period=10.0,
        actions=[LogInfo(msg=(
            "\n========================================================\n"
            "✅ GREGOR 3D BRINGUP COMPLETE - check log above for errors ✅\n"
            "   MOLA map will be saved under mola_output_dir at shutdown.\n"
            "========================================================"
        ))],
    )

    return LaunchDescription([
        rviz_arg,
        slam_arg,
        nav2_arg,
        octomap_arg,
        explore_arg,
        sweep_period_arg,
        mola_warmup_arg,
        mola_output_dir_arg,
        robot_state_publisher,
        joint_state_publisher,
        IMU_node,
        allocator_node,
        ldlidar_node,
        #thermal_Reading,
        #thermal_processor,
        ekf_node,
        lidar3d,
        mola_lio,
        octomap,
        nav2,
        explore,
        rviz,
        ready_message,
    ])
