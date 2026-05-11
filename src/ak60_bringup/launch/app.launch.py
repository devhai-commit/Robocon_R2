from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    field_color_arg = DeclareLaunchArgument(
        'field_color', default_value='red',
        description="Màu sân thi đấu: 'red' hoặc 'blue'",
        choices=['red', 'blue'],
    )

    app_manager_node = Node(
        package='ak60_bringup',
        executable='app_manager_bt.py',
        name='app_control_manager',
        output='screen',
        parameters=[{'field_color': LaunchConfiguration('field_color')}],
    )

    return LaunchDescription([
        field_color_arg,
        # Delay nhỏ để đảm bảo action servers đã sẵn sàng trước khi BT bắt đầu
        TimerAction(period=3.0, actions=[app_manager_node]),
    ])
