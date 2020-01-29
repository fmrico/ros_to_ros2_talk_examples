cmake_minimum_required(VERSION 3.5)
project(parameters_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
    rclcpp
)

add_executable(simple_param_node src/simple_param_node.cpp)
ament_target_dependencies(simple_param_node ${dependencies})

add_executable(file_param_node src/file_param_node.cpp)
ament_target_dependencies(file_param_node ${dependencies})

add_executable(reconf_param_node src/reconf_param_node.cpp)
ament_target_dependencies(reconf_param_node ${dependencies})


install(TARGETS
  simple_param_node
  file_param_node
  reconf_param_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
