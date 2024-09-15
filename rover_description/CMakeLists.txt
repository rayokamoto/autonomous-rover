cmake_minimum_required(VERSION 3.5)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
