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
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2talk_msgs/action/repeat_sentence.hpp"

// Execute:
//  ros2 run actions_ros2 server_node 
//  ros2 action list
//  ros2 action info /repeat_sentence

using std::placeholders::_1;
using std::placeholders::_2;

class RepeaterServer : public rclcpp::Node
{
public:

  using RepeatSentence = ros2talk_msgs::action::RepeatSentence;
  using GoalHandleRepeatSentence = rclcpp_action::ServerGoalHandle<RepeatSentence>;

  RepeaterServer()
  : Node("repeat_string_server")
  {
  }

  void start_server()
  {
    using namespace std::placeholders;

    repeat_sentence_action_server_ = rclcpp_action::create_server<RepeatSentence>(
      shared_from_this(),
      "repeat_sentence",
      std::bind(&RepeaterServer::handle_goal, this, _1, _2),
      std::bind(&RepeaterServer::handle_cancel, this, _1),
      std::bind(&RepeaterServer::handle_accepted, this, _1));
 
    RCLCPP_INFO(get_logger(), "Ready.");
  }

private:
  rclcpp_action::Server<RepeatSentence>::SharedPtr repeat_sentence_action_server_;
  RepeatSentence::Goal current_goal_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RepeatSentence::Goal> goal)
  {
    if (goal->times > 0) {
      current_goal_ = *goal;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRepeatSentence> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void execute(const std::shared_ptr<GoalHandleRepeatSentence> goal_handle)
  {
    rclcpp::Rate loop_rate(rclcpp::Time(current_goal_.interval).seconds());
    auto feedback = std::make_shared<RepeatSentence::Feedback>();
    auto result = std::make_shared<RepeatSentence::Result>();

    auto start = now();
    int current_times = 0;
    while (rclcpp::ok() && current_times < current_goal_.times) {
      
      if (goal_handle->is_canceling()) {
        result->total_duration = rclcpp::Time((now() - start).nanoseconds());
        result->success = false;
        
        goal_handle->canceled(result);
        
        RCLCPP_INFO(this->get_logger(), "Action Canceled");
        
        return;
      }
      
      RCLCPP_INFO(get_logger(), "%s", current_goal_.sentence.c_str());

      feedback->current_times = current_times++;
      feedback->current_duration = rclcpp::Time((now() - start).nanoseconds());
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->success = true;
      result->total_duration = rclcpp::Time((now() - start).nanoseconds());
      
      goal_handle->succeed(result);

      RCLCPP_INFO(this->get_logger(), "Action Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRepeatSentence> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&RepeaterServer::execute, this, _1), goal_handle}.detach();
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RepeaterServer>();
  
  node->start_server();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}