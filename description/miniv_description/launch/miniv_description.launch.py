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
import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_robot_description(enable_dummy):
    share_dir_path = os.path.join(get_package_share_directory('miniv_description'))
    xacro_path = ""
    if enable_dummy:
        xacro_path = os.path.join(share_dir_path, 'urdf', 'miniv_dummy.urdf.xacro')
    else:
        xacro_path = os.path.join(share_dir_path, 'urdf', 'miniv_robot.urdf.xacro')
    doc = xacro.process_file(xacro_path)
    robot_description = {"robot_description": doc.toxml()}
    return robot_description


def generate_launch_description():
    enable_dummy = LaunchConfiguration('enable_dummy', default=False)
    enable_dummy_arg = DeclareLaunchArgument(
                'enable_dummy', default_value=enable_dummy,
                description="if true, enable dummy mini-v.")
    view_model = LaunchConfiguration('view_model', default=False)
    view_model_arg = DeclareLaunchArgument(
                'view_model', default_value=view_model,
                description="if true, launch with given rviz configuration.")
    rviz = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output={
                    'stderr': 'log',
                    'stdout': 'log',
                    },
                condition=IfCondition(view_model),
                arguments=[
                    '-d', os.path.join(
                        get_package_share_directory("miniv_description"),
                        "config",
                        "miniv.rviz")])
    controller_config = os.path.join(
        get_package_share_directory("miniv_description"), "config", "controllers.yaml"
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        condition=UnlessCondition(enable_dummy),
        parameters=[generate_robot_description(False)])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[generate_robot_description(False), controller_config],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        condition=UnlessCondition(enable_dummy)
    )
    robot_state_publisher_dummy = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        condition=IfCondition(enable_dummy),
        parameters=[generate_robot_description(True)])
    control_node_dummy = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[generate_robot_description(True), controller_config],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        condition=IfCondition(enable_dummy)
    )

    return launch.LaunchDescription(
        [
            robot_state_publisher,
            robot_state_publisher_dummy,
            view_model_arg,
            enable_dummy_arg,
            rviz,
            control_node,
            control_node_dummy,
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "control",
                    "load_start_controller",
                    "joint_state_controller"],
                output="screen",
                shell=True,
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "control",
                    "load_start_controller",
                    "usv_joy_controller"],
                output="screen",
                shell=True,
            )
        ]
    )
