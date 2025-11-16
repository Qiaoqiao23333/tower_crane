# Copyright 2024
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
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # -------------------------------
    # Launch arguments
    # -------------------------------
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            description="Package where URDF file is stored.",
            default_value="tower_crane"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="can0",
            description="CAN interface (e.g. can0 for real hardware)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Use ros2_control with CANopen",
        )
    )

    # -------------------------------
    # File paths
    # -------------------------------
    description_package = LaunchConfiguration("description_package")
    can_interface_name = LaunchConfiguration("can_interface_name")

    controller_config = PathJoinSubstitution([
        FindPackageShare("tower_crane"),
        "config",
        "tower_crane_ros2_control.yaml"
    ])

    bus_config = PathJoinSubstitution([
        FindPackageShare("tower_crane"),
        "config",
        'robot_control',
        "bus.yml"
    ])

    master_config = PathJoinSubstitution([
        FindPackageShare("tower_crane"),
        "config",
        "robot_control",
        "master.dcf"
    ])

    master_bin_path = os.path.join(
        get_package_share_directory("tower_crane"),
        "config",
        "master.bin",
    )
    if not os.path.exists(master_bin_path):
        master_bin_path = ""

    # -------------------------------
    # Read URDF directly (not xacro)
    # -------------------------------
    urdf_path = os.path.join(
        get_package_share_directory("tower_crane"),
        "urdf",
        "Tower_crane.urdf"
    )

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    with open(urdf_path, "r") as infp:
        robot_description_content = infp.read()

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # -------------------------------
    # Nodes
    # -------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # The hardware plugin (canopen_ros2_control/RobotSystem) creates its own device container internally
    # DO NOT launch a separate device_container_node - it will conflict with the hardware plugin's internal one
    # Start controller_manager - the hardware plugin will initialize the device container
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controller_config],
        name="controller_manager",
    )

    # Spawn controllers after controller_manager is ready
    # Give time for the hardware plugin to boot devices and expose services
    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            )
        ]
    )

    forward_position_controller_spawner = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
            )
        ]
    )

    # -------------------------------
    # Return all nodes
    # -------------------------------
    nodes_list = [
        robot_state_publisher_node,
        controller_manager_node,  # Hardware plugin will create its own device container
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_list)

