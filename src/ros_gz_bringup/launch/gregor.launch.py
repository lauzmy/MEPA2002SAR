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

    # Kamera Node (bruker "camera_ros" eller hva du ender med for Raspberry Pi)
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        parameters=[
            {'frame_id': 'camera_link'},
            {'use_sim_time': False}
        ],
        # Remapper standard kamera topics tilbake slik de var i Gazebo Bridge
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/camera/camera_info', '/camera/camera_info')
        ]
    )

    # 5. Lokalisering & Autonomi
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_project_bringup, 'config', 'ekf.yaml'),
            {'use_sim_time': False} # Overskriver 'true' innstillingen som ligger inne i ekf.yaml!
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
        ldlidar_node,
        camera_node,
        ekf_node,
        slam,
        lidar_sweeper,
        rviz
    ])