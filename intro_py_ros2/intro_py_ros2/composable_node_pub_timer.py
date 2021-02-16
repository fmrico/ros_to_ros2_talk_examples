# Copyright 2021 Juan Carlos Manzanares Serrano
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from time import sleep
from rclpy.node import Node 
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class MyNodePublisher(Node):
    def __init__(self, name, rate):
        super().__init__(name)
        self.pub_ = self.create_publisher(String, 'chatter', 10)
        self.timer_ = self.create_timer(rate, self.timer_callback)
        self.counter_ = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Hello, world! " + str(self.counter_) + " from " + self.get_name()
        self.counter_ += 1

        self.get_logger().info('Publishing ["%s"]' % msg.data)

        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node_A = MyNodePublisher("node_pub_A", 0.5)
    node_B = MyNodePublisher("node_pub_B", 1)

    executor = MultiThreadedExecutor(2)

    executor.add_node(node_A)
    executor.add_node(node_B)

    executor.spin()
    
    node_A.destroy_node()
    node_B.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()