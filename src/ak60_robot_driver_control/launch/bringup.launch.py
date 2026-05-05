from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ak60_robot_driver_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        # Tham số URDF và controllers.yaml
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_file,
            description='Path to robot URDF/Xacro file'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=controllers_file,
            description='Path to controllers.yaml file'
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        ),

        # ROS2 Control Node
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_file],
            remappings=[
                ("~/robot_description", "/robot_description"),
            ],
            output="screen"
        ),

        # Spawner cho joint_state_broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        ),

        # Spawner cho steer_drive_controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["rocker_bogie_controller", "--controller-manager", "/controller_manager"],
            output="screen"
        ),

        # RViz2 visualization
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        ),
    ])
