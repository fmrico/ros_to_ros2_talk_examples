cmake_minimum_required(VERSION 3.5)
project(actions_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2talk_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_action
    ros2talk_msgs
)

add_executable(server_node src/server_node.cpp)
ament_target_dependencies(server_node ${dependencies})

add_executable(client_node src/client_node.cpp)
ament_target_dependencies(client_node ${dependencies})

install(TARGETS
  server_node
  client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
