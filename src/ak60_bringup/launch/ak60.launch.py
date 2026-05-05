import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # ==========================================
    # 1. KHAI BÁO BIẾN (LAUNCH ARGUMENTS)
    # ==========================================
    use_nav_arg = DeclareLaunchArgument(
        'use_nav', default_value='false',
        description='Bật true để chạy Navigation 2, tắt để chạy SLAM'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='/home/robocon/ros_ws/src/ak60_bringup/maps/my_map.yaml'
    )

    use_nav = LaunchConfiguration('use_nav')
    map_yaml = LaunchConfiguration('map')

    # ==========================================
    # 2. KHAI BÁO ĐƯỜNG DẪN CÁC PACKAGE
    # ==========================================
    pkg_description = get_package_share_directory('ak60_description')
    pkg_control     = get_package_share_directory('ak60_robot_driver_control')
    pkg_bringup     = get_package_share_directory('ak60_bringup')
    pkg_slam        = get_package_share_directory('my_robot_slam')
    pkg_nav         = get_package_share_directory('my_robot_navigation')
    pkg_ydlidar     = get_package_share_directory('ydlidar_ros2_driver')
    
    xacro_file   = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    config_file  = os.path.join(pkg_control, 'config', 'controllers.yaml')
    ekf_file     = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    lidar_config = os.path.join(pkg_ydlidar, 'params', 'ydlidar.yaml')

    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ==========================================
    # 3. KHỞI CHẠY CÁC NODE CỐT LÕI VÀ PHẦN CỨNG
    # ==========================================
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}], output='screen'
    )

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, config_file], output='screen'
    )
   
    # Tích hợp Node Lidar chạy trực tiếp cùng phần cứng
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'), 
                'launch', 
                'ydlidar.py'  # Thay bằng 'ydlidar_launch.py' nếu file của bạn tên như vậy
            )
        )
    )

    imu_node   = Node(package='ak60_robot_driver_io', executable='imu_node', output='screen')
    can_node   = Node(package='ak60_robot_driver_io', executable='can_node', output='screen')
    steer_node = Node(package='ak60_robot_driver_io', executable='serial_node', output='screen')
  
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_file]
    )

    odom_nav_node = Node(
        package='ak60_bringup', 
        executable='odom_nav_server.py',
        name='odom_nav_server',
        output='screen'
    )

    # ==========================================
    # 4. SPAWNERS (ĐIỀU KHIỂN ĐỘNG CƠ)
    # ==========================================
    jsb_spawner          = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    rocker_bogie_spawner = Node(package='controller_manager', executable='spawner', arguments=['rocker_bogie_controller'])
    arm_spawner          = Node(package='controller_manager', executable='spawner', arguments=['arm_controller', '-c', '/controller_manager'])
    # gripper_spawner      = Node(package='controller_manager', executable='spawner', arguments=['gripper_controller', '-c', '/controller_manager'])

    # ==========================================
    # 5. ĐIỀU KIỆN RẼ NHÁNH: SLAM HOẶC NAV2
    # ==========================================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'my_slam.launch.py')),
        condition=UnlessCondition(use_nav)
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'nav_bringup.launch.py')),
        launch_arguments={'map': map_yaml, 'use_sim_time': 'false'}.items(),
        condition=IfCondition(use_nav)
    )

    # ==========================================
    # 6. EVENT HANDLERS (CHUỖI KÍCH HOẠT THÔNG MINH)
    # ==========================================
    delay_jsb_after_cm      = RegisterEventHandler(OnProcessStart(target_action=controller_manager, on_start=[jsb_spawner]))
    delay_bogie_after_jsb   = RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[rocker_bogie_spawner]))
    delay_arm_after_bogie   = RegisterEventHandler(OnProcessExit(target_action=rocker_bogie_spawner, on_exit=[arm_spawner]))
    # delay_gripper_after_arm = RegisterEventHandler(OnProcessExit(target_action=arm_spawner, on_exit=[gripper_spawner]))

    return LaunchDescription([
        # use_nav_arg,
        # map_yaml_arg,

        # 1. Bật Lidar, Cảm biến, CAN/Serial và EKF ngay lập tức
        rsp_node, lidar_launch, imu_node, can_node, steer_node, ekf_node, odom_nav_node,
    
        # 2. Đợi 1 giây để phần cứng khởi tạo (đặc biệt Lidar mất thời gian quay), bật Controller Manager
        TimerAction(period=1.0, actions=[controller_manager]),

        # 3. Chuỗi kích hoạt động cơ tự động
        delay_jsb_after_cm, 
        delay_bogie_after_jsb, 
        delay_arm_after_bogie, 

        # 4. CHẠY AI DẪN ĐƯỜNG: Đợi 5 giây cho các khớp và Lidar ổn định rồi mới kích hoạt thuật toán
        # TimerAction(period=5.0, actions=[slam_launch, nav_launch]),
    ])
