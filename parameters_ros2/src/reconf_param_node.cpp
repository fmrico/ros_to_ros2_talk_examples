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

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class ReconfigurableNode : public rclcpp::Node
{
public:
  ReconfigurableNode()
  : Node("ReconfigurableNode")
  {
    declare_parameter("speed", 0.34);
    speed_ = get_parameter("speed").get_value<double>();

    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);

    parameter_event_sub_ = parameters_client_->on_parameter_event(
      std::bind(&ReconfigurableNode::on_parameter_event_callback, this, _1));
  }

private:
  double speed_;

  rclcpp::SyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  
  void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {

    for (auto & changed_parameter : event->changed_parameters) {
      const auto & type = changed_parameter.value.type;
      const auto & name = changed_parameter.name;
      const auto & value = changed_parameter.value;

      RCLCPP_INFO(get_logger(), "%s", name.c_str());

      if (name == "speed" && type == ParameterType::PARAMETER_DOUBLE) {
        RCLCPP_INFO(get_logger(), "Request for change speed from %lf to %lf",
        speed_, value.double_value);
        
        speed_ = value.double_value;
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReconfigurableNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}