import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Lấy đường dẫn động của các package
    pkg_share = get_package_share_directory('my_robot_slam')
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
    
    # 1. Đường dẫn tới file config SLAM
    slam_config = os.path.join(pkg_share, 'config', 'slam_config.yaml')
    
    # 2. Đường dẫn tới file config giao diện RViz2
    rviz_config = os.path.join(pkg_share, 'config', 'slam.rviz')
    
    # 3. Đường dẫn tới file config của YDLidar
    lidar_config = os.path.join(ydlidar_share, 'params', 'ydlidar.yaml')

    return LaunchDescription([
        
        # --- NODE YDLIDAR ---
        # Tích hợp trực tiếp Lidar vào file launch SLAM
        # Node(
        #     package='ydlidar_ros2_driver',
        #     executable='ydlidar_ros2_driver_node',
        #     name='ydlidar_ros2_driver_node',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[lidar_config]
        # ),

        # --- NODE SLAM TOOLBOX ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config, 
                {'use_sim_time': False} # Chạy với thời gian thực
            ]
        ),

        # --- NODE RVIZ2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config], # Tự động load giao diện đã lưu
            parameters=[
                {'use_sim_time': False} # Ép RViz2 đồng bộ thời gian thực
            ]
        )
    ])

