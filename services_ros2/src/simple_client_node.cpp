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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2talk_msgs/srv/reverse_string.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("reverse_string_client");

  rclcpp::Client<ros2talk_msgs::srv::ReverseString>::SharedPtr client =
    node->create_client<ros2talk_msgs::srv::ReverseString>("reverse_string");

  auto request = std::make_shared<ros2talk_msgs::srv::ReverseString::Request>();
  request->normal_sentence = "Hello World ROS2!!!!";

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Reversed: [%s] (%d)",
      result.get()->reserved_sentence.c_str(), result.get()->num_characters);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service reverse_string");
  }

  rclcpp::shutdown();

  return 0;
}