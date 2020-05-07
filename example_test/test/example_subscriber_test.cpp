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
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "example_test/ExampleSubscriber.hpp"

#include "gtest/gtest.h"

class ExampleSubscriberTest : public example_test::ExampleSubscriber
{
public:
  ExampleSubscriberTest()
  : ExampleSubscriber(),
    subscriber_ref_(subscriber_)
  {
  }

  void message_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    last_test_received_ = msg->data;

    ExampleSubscriber::message_callback(msg);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr & subscriber_ref_;
  std::string last_test_received_;
};

TEST(example_test, arrival_test)
{
  auto subscriber_node = std::make_shared<ExampleSubscriberTest>();

  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_pub = test_node->create_publisher<std_msgs::msg::String>("chatter", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(subscriber_node->get_node_base_interface());
  executor.add_node(test_node);

  ASSERT_EQ(subscriber_node->last_test_received_, "");
  ASSERT_EQ(subscriber_node->subscriber_ref_, nullptr);
  ASSERT_EQ(test_pub->get_subscription_count(), 0u);

  ASSERT_EQ(
    subscriber_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(
    subscriber_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_NE(subscriber_node->subscriber_ref_, nullptr);
  ASSERT_EQ(test_pub->get_subscription_count(), 1u);

  std_msgs::msg::String msg;
  msg.data = "one";
  test_pub->publish(msg);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(subscriber_node->last_test_received_, "one");

  msg.data = "two";
  test_pub->publish(msg);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(subscriber_node->last_test_received_, "two");

  ASSERT_EQ(
  subscriber_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(subscriber_node->subscriber_ref_, nullptr);
  ASSERT_EQ(test_pub->get_subscription_count(), 0u);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
