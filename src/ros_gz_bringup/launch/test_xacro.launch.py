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
        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'test_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
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

    slam_params_default = os.path.join(
            pkg_project_bringup, 'config', 'mapper_params_online_async.yaml'
        )
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_project_bringup, 'config', 'ekf.yaml')]
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


    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument('slam', default_value='true', description='Start slam_toolbox'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_default,
                            description='Path to slam_toolbox params yaml'),
        bridge,
        robot_state_publisher,
        lidar_sweeper,
        ekf_node,
        slam,
        rviz
    ])