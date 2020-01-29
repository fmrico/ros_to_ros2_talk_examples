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
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "ros2talk_msgs/srv/reverse_string.hpp"

using namespace std::chrono_literals;

using SharedResponse = ros2talk_msgs::srv::ReverseString::Response::SharedPtr;
using SharedFuture = std::shared_future<SharedResponse>;

class ReverseClient : public rclcpp::Node
{
public:
  ReverseClient()
  : Node("reverse_string_client"),
    counter_(0)
  {
    client_ = create_client<ros2talk_msgs::srv::ReverseString>("reverse_string");
    
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }
    
    timer_ = create_wall_timer(1s, std::bind(&ReverseClient::timer_callback, this));
  }

  void timer_callback()
  {
    RCLCPP_INFO(get_logger(), "Cycle %d", counter_);

    auto request = std::make_shared<ros2talk_msgs::srv::ReverseString::Request>();
    request->normal_sentence = std::to_string(counter_++) + " Hello World ROS2!!!!";

    pending_responses_.push_back(client_->async_send_request(request));

    // Already spinned!!!
    // auto result = client_->async_send_request(request);
    // rclcpp::spin_until_future_complete(shared_from_this(), result);

    auto it = pending_responses_.begin();
    while (it != pending_responses_.end()) {
      if (it->valid() && it->wait_for(100ms) == std::future_status::ready) {
        auto resp = it->get();

        RCLCPP_INFO(get_logger(), "Reversed: [%s] (%d)",
          resp->reserved_sentence.c_str(), resp->num_characters);

        it = pending_responses_.erase(it);
      } else {
        ++it;
      }
    }
  }

private:
  rclcpp::Client<ros2talk_msgs::srv::ReverseString>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_;

  std::list<SharedFuture> pending_responses_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReverseClient>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}