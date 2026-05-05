cmake_minimum_required(VERSION 3.12)
project(ak60_robot_driver_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

# Controller library
add_library(ak60_robot_driver_control SHARED
  src/rocker_bogie_controller.cpp
)

target_include_directories(ak60_robot_driver_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ak60_robot_driver_control
  rclcpp
  controller_interface
  pluginlib
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(
  TARGETS ak60_robot_driver_control
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf 
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
