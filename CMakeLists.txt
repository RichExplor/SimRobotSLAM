cmake_minimum_required(VERSION 3.8)
project(sim_robot_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps routes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/nav2_localization_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav2_localization_gate
)

install(
  PROGRAMS scripts/route_patrol.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME route_patrol
)

ament_package()
