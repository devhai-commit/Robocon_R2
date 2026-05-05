cmake_minimum_required(VERSION 3.14)
project(my_robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tìm các gói cơ bản
find_package(ament_cmake REQUIRED)

# Lệnh copy thư mục launch và config sang thư mục install
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

