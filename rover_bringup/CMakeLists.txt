cmake_minimum_required(VERSION 3.5)
project(rover_bringup)

find_package(ament_cmake REQUIRED)
find_package(rover_motor_controller_cpp REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
