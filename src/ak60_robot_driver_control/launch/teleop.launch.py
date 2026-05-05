from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            name="teleop_twist_keyboard",
            output="screen",
            prefix="xterm -e",  # mở terminal riêng để nhập phím
            remappings=[("/cmd_vel", "/cmd_vel")]
        )
    ])
