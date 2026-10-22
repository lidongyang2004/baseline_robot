cmake_minimum_required(VERSION 3.8)
project(baseline_robot_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY mvp_control_config mvp_mission_config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
