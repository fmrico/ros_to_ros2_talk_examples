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

class MyNodeSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub_ = self.create_subscription(String, 'chatter', self.callback, 10)
        self.sub_

    def callback(self, msg):
        self.get_logger().info("I heard " + msg.data + " in " + self.get_name())


def main(args=None):
    rclpy.init(args=args)

    node_A = MyNodeSubscriber("node_sub_A")
    node_B = MyNodeSubscriber("node_pub_B")

    executor = MultiThreadedExecutor(2)

    executor.add_node(node_A)
    executor.add_node(node_B)

    executor.spin()
    
    node_A.destroy_node()
    node_B.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()