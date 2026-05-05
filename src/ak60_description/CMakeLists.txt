cmake_minimum_required(VERSION 3.14)
project(ak60_description)

find_package(ament_cmake REQUIRED)

# Lệnh cài đặt: Copy 3 thư mục này vào share/ak60_description
install(
  DIRECTORY urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
