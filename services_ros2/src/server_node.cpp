// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2talk_msgs/srv/reverse_string.hpp"

// Execute:
//  ros2 run services_ros2 server_node 
//  ros2 service list
//  ros2 service type /reverse_string
//  ros2 interface show ros2talk_msgs/srv/ReverseString
//  ros2 service call /reverse_string ros2talk_msgs/srv/ReverseString '{normal_sentence: "Hello World ROS2!!!!"}'

void reverse_sentence(const std::shared_ptr<ros2talk_msgs::srv::ReverseString::Request> request,
          std::shared_ptr<ros2talk_msgs::srv::ReverseString::Response>      response)
{
  response->reserved_sentence = request->normal_sentence;
  std::reverse(response->reserved_sentence.rbegin(), response->reserved_sentence.rend());

  sleep(2);

  response->num_characters = response->reserved_sentence.length();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request [%s]", request->normal_sentence.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s](%d)",
    response->reserved_sentence.c_str(), response->num_characters);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("reverse_string_server");

  rclcpp::Service<ros2talk_msgs::srv::ReverseString>::SharedPtr service =
    node->create_service<ros2talk_msgs::srv::ReverseString>("reverse_string", &reverse_sentence);

  RCLCPP_INFO(node->get_logger(), "Ready.");

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
