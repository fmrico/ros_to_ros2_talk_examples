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

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("file_param_node");

  node->declare_parameter("number", 1);
  node->declare_parameter("message");
  node->declare_parameter("messages.message_1");
  node->declare_parameter("messages.message_2");
  node->declare_parameter("person_name");

  int number = node->get_parameter("number").get_value<int>();
  std::string message = node->get_parameter("message").get_value<std::string>();
  
  std::string person_name;
  node->get_parameter_or("person_name", person_name, std::string("paco"));

  std::map<std::string, std::string> msg_params;
  node->get_parameters("messages", msg_params);

  for (const auto & p : msg_params) {
    RCLCPP_INFO(node->get_logger(), "%s = %s", p.first.c_str(), p.second.c_str());
  }

  RCLCPP_INFO(node->get_logger(), "[Map] Messages [%d] for [%s] are [%s][%s]", 
    node->get_parameter("number").get_value<int>(),
    node->get_parameter("person_name").get_value<std::string>().c_str(),
    msg_params["message_1"].c_str(),
    msg_params["message_2"].c_str());
  
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}