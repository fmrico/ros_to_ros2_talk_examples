cmake_minimum_required(VERSION 3.5)
project(lifecycle_nodes_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    std_msgs
)

add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node ${dependencies})

add_executable(pub_lifecycle_node src/pub_lifecycle_node.cpp)
ament_target_dependencies(pub_lifecycle_node ${dependencies})

install(TARGETS
  simple_lifecycle_node
  pub_lifecycle_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
