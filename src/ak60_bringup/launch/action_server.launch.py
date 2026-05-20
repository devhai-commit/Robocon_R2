import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # ==========================================
    # Khai bao cac Server
    # ==========================================
    odom_nav_node = Node(
        package='ak60_bringup', 
        executable='odom_nav_server.py', 
        name='odom_nav_server', 
        output='screen'
    )

    step_climb_node = Node(
        package='ak60_bringup', 
        executable='step_climb_server.py',
        name='step_climb_server', 
        output='screen'
    )

    arm_server_node = Node(
        package='ak60_bringup', 
        executable='arm_action_server.py',
        name='arm_action_server', 
        output='screen'
    )

    tracking_server_node = Node(
        package='ak60_bringup', 
        executable='tracking_server.py',
        name='tracking_server', 
        output='screen'
    )

    lidar_align_node = Node(
        package='ak60_bringup', 
        executable='wall_alignment_server.py',
        name='alignment_server', 
        output='screen'
    )

    esp32_arm_node = Node(
        package='ak60_bringup', 
        executable='esp32_arm_action_server.py',
        name='esp32_arm_server', 
        output='screen'
    )

    k230_align_node = Node(
        package='ak60_bringup',
        executable='k230_align_server.py',
        name='k230_cam_server',
        output='screen'
    )

    return LaunchDescription([
        TimerAction(period=1.0, actions=[odom_nav_node]),
        TimerAction(period=1.0, actions=[step_climb_node]),
        TimerAction(period=1.0, actions=[arm_server_node]),
        TimerAction(period=1.0, actions=[lidar_align_node]),
        TimerAction(period=1.0, actions=[esp32_arm_node]),
        TimerAction(period=1.0, actions=[tracking_server_node]),
        TimerAction(period=1.0, actions=[k230_align_node]),
    ])
