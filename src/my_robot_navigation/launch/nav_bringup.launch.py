import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Đường dẫn tới gói của bạn và gói nav2_bringup
    pkg_nav = get_package_share_directory('my_robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 2. Lấy đường dẫn các file cấu hình
    params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    
    # Mặc định lấy bản đồ rỗng (hoặc bạn có thể chỉ định map đã quét từ SLAM)
    # Ví dụ: map_file = os.path.join(pkg_nav, 'maps', 'my_map.yaml')
    map_file = LaunchConfiguration('map', default='') 

    # 3. Khai báo các biến (Arguments)
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Đường dẫn tuyệt đối đến file bản đồ (.yaml)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Sử dụng thời gian thực cho robot thực tế')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Đường dẫn tới file cấu hình Nav2')

    # 4. Kích hoạt launch file cốt lõi của Nav2
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true'
        }.items()
    )

    # 5. Đóng gói và trả về
    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(bringup_cmd)

    return ld
