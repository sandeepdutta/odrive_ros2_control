# Copyright 2022 Factor Robotics
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

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import GroupAction

def generate_launch_description():
    slave = LaunchConfiguration("slave", default="False")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("jeeves_sim_description"),
                    "urdf",
                    "odrive_jeeves.urdf.xacro"
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_jeeves_bringup"),
            "config",
            "jeeves_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=UnlessCondition(slave),
        parameters=[robot_description, robot_controllers],
        #prefix="gdbserver localhost:2020 ",
        #arguments=['--ros-args','--log-level','DEBUG'],
        output="both",
    )

    robot_state_pub_node =  Node(
            namespace="slave",
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description]
        )

    joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            condition=UnlessCondition(slave),
            arguments=["joint_state_broadcaster", "-c", "/controller_manager","--controller-manager-timeout","50"]
        )

    robot_controller_spawner = GroupAction([
        Node(
            condition=UnlessCondition(slave),
            namespace="slave",
            package="controller_manager",
            executable="spawner",
            arguments=["diffbot_base_controller", "-c", "/controller_manager","--controller-manager-timeout","50"]
        ),
        Node (
            package="topic_tools",
            condition=UnlessCondition(slave),
            executable="relay",
            arguments=["/cmd_vel","/diffbot_base_controller/cmd_vel_unstamped"]
        )
    ])

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner
    ])
