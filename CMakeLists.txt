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

install(
  PROGRAMS scripts/web_gateway.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME web_gateway
)

install(
  FILES scripts/web_mission.py scripts/task_journal.py scripts/task_manager.py
    scripts/navigation_settings.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
