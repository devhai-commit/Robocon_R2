cmake_minimum_required(VERSION 3.12)
project(robot_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-attributes)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)

# Include thư mục chứa các file .h
include_directories(
  include
)

# Tạo thư viện plugin (shared library)
add_library(${PROJECT_NAME} SHARED
  src/hw_robot_interface.cpp
)

# Chỉ định thư mục chứa file header
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  can_msgs
)


# --- CÀI ĐẶT (INSTALL) ---

# 1. Cài đặt thư viện plugin
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# 2. Cài đặt thư mục include
install(DIRECTORY include/
  DESTINATION include
)

# Thay thế toàn bộ các lệnh cài đặt thư mục cũ bằng khối này:
install(
  DIRECTORY include launch urdf config
  DESTINATION share/${PROJECT_NAME}
)


pluginlib_export_plugin_description_file(hardware_interface robot_hw_interface.xml)
# 3. Cài đặt file mô tả plugin
install(FILES robot_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# 4. Cài đặt các thư mục chứa cấu hình và launch file
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  can_msgs
)

ament_package()
