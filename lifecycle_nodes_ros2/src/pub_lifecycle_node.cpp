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

#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class LifeCycleNodeExample : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifeCycleNodeExample()
  : rclcpp_lifecycle::LifecycleNode("lifecycle_node_example")
  {
    declare_parameter("speed", 0.34);
    pub_ = create_publisher<std_msgs::msg::Float64>("configured_speed", 100);
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
   
    speed_ = get_parameter("speed").get_value<double>();
   
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    pub_->on_activate();
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    
    pub_->on_deactivate();
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    
    pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    
    pub_.reset();
    
    return CallbackReturnT::SUCCESS;
  }

    CallbackReturnT on_error(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }
  
  void do_work() 
  {
    if (pub_->is_activated()) {
      std_msgs::msg::Float64 msg;
      msg.data = speed_;

      pub_->publish(msg);
    }
  }


private:
  double speed_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LifeCycleNodeExample>();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->do_work();

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  

  rclcpp::shutdown();

  return 0;
}