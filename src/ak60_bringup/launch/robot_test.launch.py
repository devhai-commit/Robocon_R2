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
    # 1. KHAI BÁO ĐƯỜNG DẪN CÁC PACKAGE
    # ==========================================
    pkg_description = get_package_share_directory('ak60_description')
    pkg_control     = get_package_share_directory('ak60_robot_driver_control')
    pkg_bringup     = get_package_share_directory('ak60_bringup')
    
    # [THÊM MỚI] Đường dẫn package của RealSense
    pkg_realsense   = get_package_share_directory('realsense2_camera')
    
    xacro_file   = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    config_file  = os.path.join(pkg_control, 'config', 'controllers.yaml')
    ekf_file     = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ==========================================
    # 2. KHỞI CHẠY CÁC NODE CỐT LÕI VÀ PHẦN CỨNG
    # ==========================================
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}], 
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, config_file], 
        output='screen'
    )

    # Các node giao tiếp phần cứng
    imu_node   = Node(package='ak60_robot_driver_io', executable='imu_node', output='screen')
    can_node   = Node(package='ak60_robot_driver_io', executable='can_node', output='screen')
    steer_node = Node(package='ak60_robot_driver_io', executable='serial_node', output='screen')
  
    # Bộ lọc EKF để kết hợp Odometry từ bánh xe và IMU
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_file]
    )

    # Khởi chạy Camera RealSense (Bật Align Depth, Tắt IR để tiết kiệm USB)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_realsense, 'launch', 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false', 
            'initial_reset': 'true'
        }.items()
    )

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

    # ==========================================
    # 3. SPAWNERS (NẠP BỘ ĐIỀU KHIỂN)
    # ==========================================
    jsb_spawner          = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    rocker_bogie_spawner = Node(package='controller_manager', executable='spawner', arguments=['rocker_bogie_controller'])
    arm_spawner          = Node(package='controller_manager', executable='spawner', arguments=['arm_controller', '-c', '/controller_manager'])
    
    # ==========================================
    # 4. CHUỖI KÍCH HOẠT THÔNG MINH
    # ==========================================
    delay_jsb_after_cm      = RegisterEventHandler(OnProcessStart(target_action=controller_manager, on_start=[jsb_spawner]))
    delay_bogie_after_jsb   = RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[rocker_bogie_spawner]))
    delay_arm_after_bogie   = RegisterEventHandler(OnProcessExit(target_action=rocker_bogie_spawner, on_exit=[arm_spawner]))

    return LaunchDescription([
        # 1. Bật URDF, Cảm biến, CAN/Serial, EKF và RealSense ngay từ đầu
        rsp_node, imu_node, can_node, steer_node, ekf_node, realsense_launch,
      
        # 2. Bật Controller Manager sau 1 giây
        TimerAction(period=1.0, actions=[controller_manager]),

        # 3. Chuỗi kích hoạt động cơ tự động
        delay_jsb_after_cm, 
        delay_bogie_after_jsb, 
        delay_arm_after_bogie,

        TimerAction(period=1.0, actions=[odom_nav_node]),
        TimerAction(period=1.0, actions=[step_climb_node]),
        TimerAction(period=1.0, actions=[arm_server_node]),
        TimerAction(period=1.0, actions=[tracking_server_node])
    ])
