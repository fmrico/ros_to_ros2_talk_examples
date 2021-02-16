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
from std_msgs.msg import String

class MyNodePublisher(Node):
    def __init__(self):
        super().__init__("composable_node_pub")
        self.pub_ = self.create_publisher(String, 'chatter', 10)
        self.counter = 0

    def do_work(self):
        msg = String()
        msg.data = "Hello, world! " + str(self.counter)
        self.counter += 1

        self.get_logger().info('Publishing ["%s"]' % msg.data)

        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MyNodePublisher()

    loop_rate = 0.5
    while rclpy.ok():
        node.do_work()
        sleep(loop_rate)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()