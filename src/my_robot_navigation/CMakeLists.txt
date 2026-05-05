cmake_minimum_required(VERSION 3.14)
project(my_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CHỈ GIỮ LẠI CÔNG CỤ BUILD NÀY
find_package(ament_cmake REQUIRED)

# Cài đặt các thư mục vào ROS 2 Install space
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()