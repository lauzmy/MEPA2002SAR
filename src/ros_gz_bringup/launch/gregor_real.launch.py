import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction, LogInfo
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit



def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('ros_gz_bringup')
    pkg_gregor_description = get_package_share_directory('gregor_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # 1. Last URDF for den fysiske roboten direkte (ingen xacro-konvertering).
    urdf_file = os.path.join(pkg_gregor_description, 'urdf', 'gregor.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 2. Argumenter du kan bruke i terminalen
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Open RViz')
    slam_arg = DeclareLaunchArgument('slam', default_value='true', description='Run SLAM')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='true', description='Run Nav2')
    explore_arg = DeclareLaunchArgument('explore', default_value='true', description='Run autonomous exploration')

    # 3. TF Tree publisering
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': robot_desc}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    allocator_node = Node(
        package='ros_gz_application',
        executable='allocator',
        name='allocator_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    IMU_node = Node(
        package='ros_gz_application',
        executable='IMU',
        name='imu_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'i2c_bus': 4,
        }]
    )

    # 4. Ekte sensorer
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyAMA0'},
            {'topic_name': '/scan'},
            {'frame_id': 'laser'},
            {'lidar_type': 'LD06'}
        ]
    )

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_node',
        parameters=[
            {'video_device': '/dev/video0'},
            {'frame_id': 'camera_link'},
            {'image_width': 640},
            {'image_height': 480},
            {'framerate': 25.0},
            {'use_sim_time': False}
        ],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ]
    )

    thermal_Reading = Node(
        package='ros_gz_application',
        executable='thermal_reading',
        name='thermal_Reading',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    thermal_processor = Node(
        package='ros_gz_application',
        executable='thermal_processing',
        name='thermal_processor',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    collision_avoidance = Node(
        package='ros_gz_application',
        executable='collision_avoidance',
        name='collision_avoidance',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 5. Lokalisering & Autonomi
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_project_bringup, 'config', 'IRL', 'ekf_imu.yaml'),
            {'use_sim_time': False}
        ]
    )

    slam_params_default = os.path.join(pkg_project_bringup, 'config', 'IRL', 'mapper_params_online_async.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_default,
            'use_sim_time': 'False'
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(pkg_project_bringup, 'config', 'IRL', 'nav2_params_smac2D.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params_file
        }.items(),
    )

    nav2 = TimerAction(
        period=5.0,
        actions=[nav2_launch],
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    explore_lite_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            os.path.join(pkg_project_bringup, 'config', 'sim', 'explore.yaml'),
            {'use_sim_time': False},
        ],
    )
    explore = TimerAction(
        period=12.0,
        actions=[explore_lite_node],
        condition=IfCondition(LaunchConfiguration('explore')),
    )

    exploration_monitor = Node(
        package='ros_gz_application',
        executable='exploration_monitor',
        name='exploration_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'maps_root': '/home/ubuntu/ros2_ws/maps',
            'map_name': 'gregor_map',
            'idle_timeout': 15.0,
            'startup_grace': 30.0,
            'push_script': '/home/ubuntu/push_map.sh',
        }],
    ) 

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'test_robot.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ready_message = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="\n"
                        "========================================================\n"
                        "✅ OPPSTART FULLFØRT (gregor_description URDF)         ✅\n"
                        "========================================================")
        ]
    )

    return LaunchDescription([
        rviz_arg,
        slam_arg,
        nav2_arg,
        explore_arg,
        robot_state_publisher,
        joint_state_publisher_node,
        IMU_node,
        allocator_node,
        ldlidar_node,
        thermal_Reading,
        thermal_processor,
        collision_avoidance,
        ekf_node,
        slam,
        nav2,
        explore,
        exploration_monitor,
        ready_message,
    ])
