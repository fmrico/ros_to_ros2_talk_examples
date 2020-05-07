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

#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "example_test/ExampleSubscriber.hpp"

namespace example_test
{
using std::placeholders::_1;

ExampleSubscriber::ExampleSubscriber()
: LifecycleNode("example_test")
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExampleSubscriber::on_activate(const rclcpp_lifecycle::State &)
{
  subscriber_ = create_subscription<std_msgs::msg::String>("chatter", 10,
      std::bind(&ExampleSubscriber::message_callback, this, _1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExampleSubscriber::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_ = nullptr;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
ExampleSubscriber::message_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "Msg received: " << msg->data);
}

}  // namespace example_test
