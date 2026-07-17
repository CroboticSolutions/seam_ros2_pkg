cmake_minimum_required(VERSION 3.8)
project(seam_ros2_pkg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf worlds srdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/smoke_fanuc_real.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
