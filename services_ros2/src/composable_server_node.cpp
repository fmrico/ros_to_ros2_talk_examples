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

// Execute:
//  ros2 run services_ros2 server_node 
//  ros2 service list
//  ros2 service type /reverse_string
//  ros2 interface show ros2talk_msgs/srv/ReverseString
//  ros2 service call /reverse_string ros2talk_msgs/srv/ReverseString '{normal_sentence: "Hello World ROS2!!!!"}'

using std::placeholders::_1;
using std::placeholders::_2;

class ReverseServer : public rclcpp::Node
{
public:
  ReverseServer()
  : Node("reverse_string_server")
  {
    service_ = create_service<ros2talk_msgs::srv::ReverseString>(
      "reverse_string", std::bind(&ReverseServer::reverse_sentence, this, _1, _2));

     RCLCPP_INFO(get_logger(), "Ready.");
  }

  void reverse_sentence(const std::shared_ptr<ros2talk_msgs::srv::ReverseString::Request> request,
          std::shared_ptr<ros2talk_msgs::srv::ReverseString::Response>      response)
  {
    response->reserved_sentence = request->normal_sentence;
    std::reverse(response->reserved_sentence.rbegin(), response->reserved_sentence.rend());

    response->num_characters = response->reserved_sentence.length();

    RCLCPP_INFO(get_logger(), "Incoming request [%s]", request->normal_sentence.c_str());
    RCLCPP_INFO(get_logger(), "sending back response: [%s](%d)",
      response->reserved_sentence.c_str(), response->num_characters);
  }

private:
  rclcpp::Service<ros2talk_msgs::srv::ReverseString>::SharedPtr service_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReverseServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}