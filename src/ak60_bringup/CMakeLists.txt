cmake_minimum_required(VERSION 3.14)
project(ak60_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# Cài đặt r2_bt như một Python package để import được
ament_python_install_package(r2_bt PACKAGE_DIR scripts/r2_bt)

# Lệnh cài đặt để ROS 2 tìm thấy file Launch của bạn
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odom_nav_server.py
  scripts/step_climb_server.py
  scripts/tracking_server.py
  scripts/arm_action_server.py
  scripts/esp32_arm_action_server.py
  scripts/wall_alignment_server.py
  scripts/app_manager_bt.py
  
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveArm.action"
  "action/FollowTarget.action"
  "action/ClimbStep.action"
  "action/WallAlignment.action"
  "action/ArmSequence.action"

  DEPENDENCIES action_msgs
)

ament_package()
