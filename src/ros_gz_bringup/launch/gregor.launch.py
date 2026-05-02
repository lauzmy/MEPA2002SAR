import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('ros_gz_bringup')
    pkg_project_description = get_package_share_directory('ros_gz_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # 1. Behandle URDF for den fysiske roboten (Xacro -> URDF)
    # På den ekte roboten trenger du ikke lage en SDF-fil til Gazebo.
    xacro_file = os.path.join(pkg_project_description, 'models', 'xacro_test', 'test.urdf.xacro')
    robot_desc = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')

    # 2. Argumenter du kan bruke i terminalen (f.eks "ros2 launch ros_gz_bringup gregor.launch.py rviz:=false")
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Open RViz')
    slam_arg = DeclareLaunchArgument('slam', default_value='true', description='Run SLAM')
    
    # 3. TF Tree publisering
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': False},  # VELDIG VIKTIG PÅ EKTE ROBOT
            {'robot_description': robot_desc}
        ]
    )

    allocator_node = Node(
        package='ros_gz_application',
        executable='allocator',
        name='allocator_node',
        output='screen',
        parameters=[{'use_sim_time': False}]

    )
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_footprint', 'laser']
    )

    IMU_node = Node(
        package='ros_gz_application',
        executable='IMU',
        name='imu_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 4. Ekte Sensorer 
    # Vår nye LiDAR node (pass på at typen, f.eks LD19, stemmer med din lidar)
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyAMA0'},  # Bytt ut basert på porten til Pi-en
            {'topic_name': '/test_robot/scan'}, # Leser samme topic som simuleringen
            {'frame_id': 'lidar_link'},       # Navnet på linken i din xacro
            {'lidar_type': 'LD06'}
        ]
    )

    # Kamera Node for USB-kamera (bruker usb_cam)
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_node',
        parameters=[
            {'video_device': '/dev/video0'}, # Standard for første USB-kamera
            {'frame_id': 'camera_link'},
            {'image_width': 640},            # Tilpass oppløsningen til ditt kamera
            {'image_height': 480},
            {'framerate': 25.0},
            {'use_sim_time': False}
        ],
        remappings=[
            ('/image_raw', '/camera/image_raw'),     # Gjør at topics fra usb_cam matcher
            ('/camera_info', '/camera/camera_info')  # forventningene fra Gazebo/resten av koden
        ]
    )

    # 5. Lokalisering & Autonomi
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_project_bringup, 'config', 'ekf_imu.yaml'),
            {'use_sim_time': False} # Overskriver 'true' innstillingen som ligger inne i ekf_imu.yaml!
        ]
    )
    
    slam_params_default = os.path.join(pkg_project_bringup, 'config', 'mapper_params_online_async.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_default,
            'use_sim_time': 'False' # Sørger for at SLAM ikke venter på Gazebo-klokken
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # 6. Din egendefinerte kode
    lidar_sweeper = Node(
        package='ros_gz_application',
        executable='lidar_sweeper',
        name='lidar_sweeper',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'test_robot.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        slam_arg,
        robot_state_publisher,
        IMU_node,
        allocator_node,
        ldlidar_node,
        camera_node,
        ekf_node,
        slam,
        lidar_sweeper,
        laser_tf_node,
    ])