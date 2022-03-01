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

#include <chrono>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("simple_node");
  node->declare_parameter("rate");

  std::chrono::nanoseconds rate_ = 200ms;

  double rate = 1.0 / std::chrono::duration<double>(rate_).count();
 
  std::cerr << "default rate = " << rate << std::endl;
 
  node->get_parameter_or("rate", rate, rate);
 
  std::cerr << 1.0 / rate << std::endl;

  std::chrono::nanoseconds rate2_ =  std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / rate));
  
  // std::chrono::duration<double, std::ratio<1>>(1.0 / rate)

  if (rate_ == rate2_) {
    std::cerr << "Son iguales" << std::endl;
  } else {
    std::cerr << "Son diferentes" << std::endl;
  }
  //double rate2 = 1.0 / std::chrono::duration<double>(rate2_).count();



  std::cerr << "final rate = " << rate << std::endl;

  //rate_ = rate2_;

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}