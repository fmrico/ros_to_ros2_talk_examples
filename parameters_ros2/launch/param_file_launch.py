# Copyright 2020 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

  stdout_linebuf_envvar = SetEnvironmentVariable(
    'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
 
  pkg_dir = get_package_share_directory('parameters_ros2')
  config_dir = os.path.join(pkg_dir, 'config')
  config_file = os.path.join(config_dir, 'config_2.yaml')

  file_param_node_cmd = Node(
    package='parameters_ros2',
    node_executable='file_param_node',
    output='screen',
    parameters=[config_file])
  
  ld = LaunchDescription()
  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(file_param_node_cmd)

  return ld
