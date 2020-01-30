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


// You can execute: 
//  $ ros2 run parameters_ros2 simple_param_node --ros-args -p message:=hola
//  $ ros2 param list
//  $ simple_param_node
//  $ ros2 param get /simple_param_node message

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("simple_param_node");

  node->declare_parameter("number", 1);
  node->declare_parameter("message", "Ey");
  node->declare_parameter("person_name", "Dude");

  int number = node->get_parameter("number").get_value<int>();
  std::string message = node->get_parameter("message").get_value<std::string>();
  
  std::string person_name;
  node->get_parameter_or("person_name", person_name, std::string("paco"));

  RCLCPP_INFO(node->get_logger(), "[Separate] Message [%d] for [%s] is [%s]", number,
    person_name.c_str(), message.c_str());

  node->set_parameter({"person_name", "pepe"});
  
  auto v_params = node->get_parameters({"number", "message", "person_name"});
  RCLCPP_INFO(node->get_logger(), "[Join] Message [%d] for [%s] is [%s]", 
    v_params[0].get_value<int>(),
    v_params[1].get_value<std::string>().c_str(),
    v_params[2].get_value<std::string>().c_str());

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}