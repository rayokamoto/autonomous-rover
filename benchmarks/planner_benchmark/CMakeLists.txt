cmake_minimum_required(VERSION 3.5)
project(planner_benchmark NONE)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
