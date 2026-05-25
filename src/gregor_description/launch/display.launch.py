"""Display Gregor in RViz with robot_state_publisher + joint_state_publisher_gui."""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("gregor_description")
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "gregor.urdf"])
    rviz_path = PathJoinSubstitution([pkg_share, "rviz", "display.rviz"])

    robot_description = {"robot_description": Command(["cat ", urdf_path])}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_path],
            output="screen",
        ),
    ])
