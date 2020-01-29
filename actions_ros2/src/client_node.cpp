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
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class RepeaterClient : public rclcpp::Node
{
public:

  using RepeatSentence = ros2talk_msgs::action::RepeatSentence;
  using GoalHandleRepeatSentence = rclcpp_action::ClientGoalHandle<RepeatSentence>;

  RepeaterClient()
  : Node("repeat_string_client")
  {

  }

  void call_server() 
  {
    repeat_sentence_client_ptr_ = rclcpp_action::create_client<RepeatSentence>(
      shared_from_this(), "repeat_sentence");
    
    if (!this->repeat_sentence_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = RepeatSentence::Goal();
    goal_msg.sentence = "Hello from ROS2!!";
    goal_msg.times = 5;
    goal_msg.interval = rclcpp::Time(1, 0);

    auto send_goal_options = rclcpp_action::Client<RepeatSentence>::SendGoalOptions();

    send_goal_options.feedback_callback =
      std::bind(&RepeaterClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&RepeaterClient::result_callback, this, _1);
    
    auto goal_handle_future = repeat_sentence_client_ptr_->async_send_goal(
      goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "send_goal failed");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(
        get_logger(), "ExecutorClient: Execution was rejected by the action server");
      return;
    }
  }

private:
  rclcpp_action::Client<RepeatSentence>::SharedPtr repeat_sentence_client_ptr_;
  
  void feedback_callback(
    GoalHandleRepeatSentence::SharedPtr,
    const std::shared_ptr<const RepeatSentence::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "[%d] elapsed %lf",
      feedback->current_times, rclcpp::Time(feedback->current_duration).seconds());
  }

  void result_callback(const GoalHandleRepeatSentence::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }

    if (result.result->success) {
      RCLCPP_INFO(get_logger(), "Success: elapsed %lf", 
        rclcpp::Time(result.result->total_duration).seconds());
    } else {
      RCLCPP_INFO(get_logger(), "Failed: elapsed %lf", 
        rclcpp::Time(result.result->total_duration).seconds());
    }
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RepeaterClient>();
  
  node->call_server();
  
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}