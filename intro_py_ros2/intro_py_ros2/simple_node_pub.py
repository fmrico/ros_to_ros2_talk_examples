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

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('simple_node_pub')
    publisher = node.create_publisher(String, 'chatter', 10)

    msg = String()
    counter = 0

    loop_rate = 0.5
    while rclpy.ok():
        msg.data = "Hello, world! " + str(counter)
        counter += 1

        node.get_logger().info('Publishing ["%s"]' % msg.data)

        publisher.publish(msg)

        sleep(0.5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()