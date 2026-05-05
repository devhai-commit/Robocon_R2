import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_hw_interface')
    
    # CHÚ Ý: Đảm bảo tên file URDF và YAML khớp với tên thực tế trong thư mục của bạn
    urdf_file = os.path.join(pkg_dir, 'urdf', 'test_robot.urdf')
    config_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')

    # Đọc nội dung file URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # =======================================================
    # 1. KHỞI CHẠY ROBOT STATE PUBLISHER (Cho RViz 2)
    # =======================================================
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # =======================================================
    # 2. KHỞI CHẠY RVIZ 2
    # =======================================================
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen'
    # )

    # =======================================================
    # 3. KHỞI CHẠY ROS2_CONTROL
    # =======================================================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, config_file],
        output='screen'
    )

    # =======================================================
    # 4. CÁC SPAWNER BỘ ĐIỀU KHIỂN
    # =======================================================
    jsb_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['joint_state_broadcaster'],
    )
    
    vel_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['velocity_controller'],
    )
    
    pos_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['position_controller'],
    )

    return LaunchDescription([
        rsp_node,
        # rviz_node,
        controller_manager,
        # Đẩy delay ra thêm một chút để RViz và RSP kịp khởi động
        TimerAction(period=2.0, actions=[jsb_spawner]),
        TimerAction(period=3.0, actions=[vel_spawner]),
        TimerAction(period=4.0, actions=[pos_spawner]),
    ])

