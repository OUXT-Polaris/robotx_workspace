# Copyright (c) 2020 OUXT Polaris
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
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    joy_param_file = LaunchConfiguration(
        'joy_param_file',
        default=os.path.join(
            get_package_share_directory('joy_to_twist'),
            'config', 'joy.yaml'))
    description = LaunchDescription([
        Node(
            package='joy',
            node_executable='joy_node',
            node_name='joy_node',
            parameters=[joy_param_file],
            output='screen'),
        Node(
            package='joy_to_twist',
            node_executable='joy_to_twist_node',
            node_name='joy_to_twist_node',
            output='screen'),
    ])
    return description
