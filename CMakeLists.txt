cmake_minimum_required(VERSION 3.8)
project(sim_robot_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
