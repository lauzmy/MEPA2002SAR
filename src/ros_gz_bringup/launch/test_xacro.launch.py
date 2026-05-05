import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('ros_gz_bringup')
    pkg_project_description = get_package_share_directory('ros_gz_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_example_gazebo = get_package_share_directory('ros_gz_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # World xacro -> sdf (samme stil som roboten)
    world_xacro_file = os.path.join(pkg_example_gazebo, 'worlds', 'test_xacro.sdf.xacro')
    world_sdf_content = subprocess.check_output(['xacro', world_xacro_file]).decode('utf-8')
    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as world_file:
        world_file.write(world_sdf_content)
        world_file_path = world_file.name

    # Robot xacro -> urdf -> sdf
    xacro_file = os.path.join(pkg_project_description, 'models', 'xacro_test', 'test.urdf.xacro')
    model_sdf_file = os.path.join(pkg_project_description, 'models', 'xacro_test', 'model.sdf')

    urdf_content = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as urdf_file:
        urdf_file.write(urdf_content)
        urdf_file_path = urdf_file.name

    sdf_content = subprocess.check_output(['gz', 'sdf', '-p', urdf_file_path]).decode('utf-8')
    os.unlink(urdf_file_path)

    with open(model_sdf_file, 'w') as f:
        f.write(sdf_content)

    robot_desc = urdf_content

    models_path = os.path.join(pkg_project_description, 'models')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, os.pathsep, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    # Lanseringsargumenter (Terminal-parametere)
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.')
    slam_arg = DeclareLaunchArgument('slam', default_value='true', description='Start slam_toolbox')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='true', description='Run Nav2')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time')

    slam_params_default = os.path.join(
            pkg_project_bringup, 'config', 'sim', 'mapper_params_online_async.yaml'
        )
    slam_params_arg = DeclareLaunchArgument('slam_params_file', default_value=slam_params_default,
                            description='Path to slam_toolbox params yaml')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}, {'robot_description': robot_desc}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'sim' , 'test_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'sim', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    lidar_sweeper = Node(
        package='ros_gz_application',
        executable='lidar_sweeper',
        name='lidar_sweeper',
        output='screen'
    )

    laser_assembler_node = Node(
        package='ros_gz_application',
        executable='laser_assembler',
        name='tilt_laser_assembler',
        output='screen',
        parameters=[{
            'use_sim_time': True # VIKTIG for at TF2 og klokken skal synkroniseres med Gazebo
        }],
        remappings=[
            # Noden din lytter til '/scan', men Gazebo-bridgen publiserer til '/test_robot/scan'
            ('/scan', '/test_robot/scan') 
        ]
    )

    YOLO_node = Node(
        package='ros_gz_application',
        executable='YOLO',
        name='yolo_coco_node',
        output='screen',
    )
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_project_bringup, 'config', 'sim', 'ekf.yaml')]
    )
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # Finn stien til Nav2 konfigurasjonsfilen for simulering
    nav2_params_file = os.path.join(pkg_project_bringup, 'config', 'sim', 'nav2_params_protomota.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )



    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        
        # Inkluder disse deklarerte argumentene
        rviz_arg,
        slam_arg,
        nav2_arg,
        use_sim_time_arg,
        slam_params_arg,

        bridge,
        robot_state_publisher,
        lidar_sweeper,
        laser_assembler_node,
        ekf_node,
        slam,
        nav2,
        rviz,
        YOLO_node
    ])