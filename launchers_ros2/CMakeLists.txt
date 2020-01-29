cmake_minimum_required(VERSION 3.5)
project(launchers_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
