from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="ak60_robot_driver_io",
            executable="can_node",
            output="screen"
        ),
        Node(
            package="ak60_robot_driver_io",
            executable="serial_node",
            output="screen"
        ),
    ])
