import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction, LogInfo

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
    nav2_arg = DeclareLaunchArgument('nav2', default_value='true', description='Run Nav2')
    
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
            {'frame_id': 'laser'},       # Navnet på linken i din xacro
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

    #Termokamera node for å lese rå termiske bilder fra /dev/video2
    thermal_converter = Node(
        package='ros_gz_application',
        executable='thermal_converter',
        name='thermal_converter',
        output='screen',
        parameters=[
            {'frame_rate': 9.0},
            {'show_preview': True},
            {'preview_scale': 4.0},
            {'use_sim_time': False}
        ]
    )

    thermal_processor = Node(
        package = 'ros_gz_application',
        executable = 'thermal_processing',
        name = 'thermal_processor',
        output = 'screen',
        parameters=[
                {'min_temp_celsius': 20.0},
                {'max_temp_celsius': 100.0},
                {'temp_bins': [20.0, 35.0, 40.0, 50.0, 60.0]},
            {'num_top_spots': 3},
            {'min_area_pixels': 50},
            {'gaussian_blur_kernel': 3}
        ]
    )
        

    # 5. Lokalisering & Autonomi
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_project_bringup, 'config', 'IRL', 'ekf_imu.yaml'),
            {'use_sim_time': False} # Overskriver 'true' innstillingen som ligger inne i ekf_imu.yaml!
        ]
    )
    
    slam_params_default = os.path.join(pkg_project_bringup, 'config', 'IRL', 'mapper_params_online_async.yaml')
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

    # Finn stien til nav2_bringup
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Finn stien til Nav2 konfigurasjonsfilen
    # OBS: Sørg for at filnavnet matcher din faktiske config-fil
    nav2_params_file = os.path.join(pkg_project_bringup, 'config', 'IRL', 'nav2_params_smac2D.yaml')

    # Nav2 lanseringsbeskrivelse
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params_file
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
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
    # Venter 5 sekunder før den printer en bekreftelse 
    ready_message = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="\n"
                        "========================================================\n"
                        "✅ OPPSTART FULLFØRT! (Sjekk loggen over for evt. feil) ✅\n"
                        "========================================================")
        ]
    )

    return LaunchDescription([
        rviz_arg,
        slam_arg,
        nav2_arg,
        robot_state_publisher,
        joint_state_publisher_node,
        IMU_node,
        allocator_node,
        ldlidar_node,
        ekf_node,
        slam,
        nav2,
        ready_message
    ])

