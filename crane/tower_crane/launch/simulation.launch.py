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

"""
Unified simulation launch file for tower crane.

This launch file integrates:
- Fake CANopen slaves (canopen_fake_slaves)
- CANopen master (canopen_core)
- ROS2 control with mock hardware
- Robot state publisher
- Controllers (joint_state_broadcaster, forward_position_controller)
- Optional RViz visualization
- Optional joint_state_publisher_gui (currently disabled)

Usage:
    # Basic simulation with RViz
    ros2 launch tower_crane simulation.launch.py

    # Simulation without RViz
    ros2 launch tower_crane simulation.launch.py use_rviz:=false

    # Custom CAN interface
    ros2 launch tower_crane simulation.launch.py can_interface_name:=vcan0
"""

import os
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

_BUS_CONFIG_PATH_PLACEHOLDER = "@BUS_CONFIG_PATH@"


def _prepare_bus_config(share_dir: str) -> str:
    """Return a bus.yml path usable by canopen_core.

    The checked-in bus.yml contains a @BUS_CONFIG_PATH@ placeholder.
    canopen_core expects a real path, so we patch it at runtime.

    Instead of writing to /tmp (which breaks when running nodes manually), we
    write the patched file to a stable location under ~/.ros.
    """
    source_path = os.path.join(share_dir, "config", "robot_control", "bus.yml")
    with open(source_path, "r", encoding="utf-8") as infp:
        content = infp.read()

    # If no placeholder remains, use the file in-place.
    if _BUS_CONFIG_PATH_PLACEHOLDER not in content:
        return source_path

    expected_config_path = os.path.join(share_dir, "config", "robot_control")
    patched_content = content.replace(_BUS_CONFIG_PATH_PLACEHOLDER, expected_config_path)

    ros_home = os.environ.get("ROS_HOME", os.path.join(os.path.expanduser("~"), ".ros"))
    out_dir = os.path.join(ros_home, "tower_crane")
    os.makedirs(out_dir, exist_ok=True)

    out_path = os.path.join(out_dir, "bus.patched.yml")
    with open(out_path, "w", encoding="utf-8") as outfp:
        outfp.write(patched_content)

    return out_path


def _load_robot_description() -> ParameterValue:
    """Load robot description from URDF file."""
    share_dir = get_package_share_directory("tower_crane")
    urdf_file = os.path.join(share_dir, "urdf", "Tower_crane.urdf")
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    with open(urdf_file, "r", encoding="utf-8") as infp:
        return ParameterValue(infp.read(), value_type=str)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="CAN interface to use (vcan0 for mock simulation, can0 for real CAN)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz visualization (true/false)",
        ),
        # DeclareLaunchArgument(
        #     "use_joint_state_publisher_gui",
        #     default_value="true",
        #     description="Start joint_state_publisher_gui (true/false)",
        # ),
        DeclareLaunchArgument(
            "use_robot_state_publisher",
            default_value="true",
            description="Start robot_state_publisher (true/false). Set to false if launched from another launch file that already includes RSP.",
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        can_interface_name = LaunchConfiguration("can_interface_name").perform(context)
        use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() == "true"
        # use_joint_state_publisher_gui = (
        #     LaunchConfiguration("use_joint_state_publisher_gui").perform(context).lower() == "true"
        # )
        use_robot_state_publisher = (
            LaunchConfiguration("use_robot_state_publisher").perform(context).lower() == "true"
        )

        share_dir = get_package_share_directory("tower_crane")
        rviz_config_file = os.path.join(share_dir, "urdf.rviz")

        # Load robot description
        robot_description = {"robot_description": _load_robot_description()}

        # Prepare CANopen configuration
        master_config = os.path.join(share_dir, "config", "robot_control", "master.dcf")
        master_bin = os.path.join(share_dir, "config", "robot_control", "master.bin")
        bus_config = _prepare_bus_config(share_dir)
        slave_config = os.path.join(share_dir, "config", "robot_control", "DSY-C.EDS")

        # Fake CANopen slaves (must start first)
        slave_launch = os.path.join(
            get_package_share_directory("canopen_fake_slaves"),
            "launch",
            "cia402_slave.launch.py",
        )

        slave_node_1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slave_launch),
            launch_arguments={
                "node_id": "2",
                "node_name": "slave_node_1",
                "slave_config": slave_config,
                "can_interface_name": can_interface_name,
            }.items(),
        )

        slave_node_2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slave_launch),
            launch_arguments={
                "node_id": "1",
                "node_name": "slave_node_2",
                "slave_config": slave_config,
                "can_interface_name": can_interface_name,
            }.items(),
        )

        slave_node_3 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slave_launch),
            launch_arguments={
                "node_id": "3",
                "node_name": "slave_node_3",
                "slave_config": slave_config,
                "can_interface_name": can_interface_name,
            }.items(),
        )

        # CANopen master (delayed to ensure slaves are ready)
        device_container = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("canopen_core"),
                    "launch",
                    "canopen.launch.py",
                )
            ),
            launch_arguments={
                "master_config": master_config,
                "master_bin": master_bin if os.path.exists(master_bin) else "",
                "bus_config": bus_config,
                "can_interface_name": can_interface_name,
                "master.can_interface_name": can_interface_name,
            }.items(),
        )

        # ROS2 control node with mock hardware
        controller_config = os.path.join(share_dir, "config", "tower_crane_ros2_control.yaml")
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description],
            arguments=["--ros-args", "--params-file", controller_config],
            output="screen",
        )

        # Controllers spawners
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

        forward_position_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "forward_position_controller",
                "--controller-manager",
                "/controller_manager",
            ],
        )

        # Robot state publisher
        # Note: robot_state_publisher needs joint_states to compute TF transforms.
        # We delay it slightly to ensure joint_state_broadcaster is ready.
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )

        # Build launch list
        nodes_to_launch = [
            slave_node_1,
            slave_node_2,
            slave_node_3,
        ]

        # Delayed nodes
        delayed_device_container = TimerAction(period=0.1, actions=[device_container])
        delayed_control_node = TimerAction(period=3.0, actions=[control_node])
        # Start joint_state_broadcaster before forward_position_controller
        # This ensures joint_states are available for robot_state_publisher
        delayed_joint_state_broadcaster = TimerAction(
            period=5.0,
            actions=[joint_state_broadcaster_spawner],
        )
        delayed_forward_position_controller = TimerAction(
            period=5.5,
            actions=[forward_position_controller_spawner],
        )
        
        # Conditionally add robot_state_publisher
        # Start it early (it can work without joint_states initially)
        # but we delay it slightly to ensure URDF is loaded properly
        if use_robot_state_publisher:
            delayed_robot_state_publisher = TimerAction(period=0.5, actions=[robot_state_publisher_node])
            nodes_to_launch.extend([
                delayed_robot_state_publisher,
                delayed_device_container,
                delayed_control_node,
                delayed_joint_state_broadcaster,
                delayed_forward_position_controller,
            ])
        else:
            nodes_to_launch.extend([
                delayed_device_container,
                delayed_control_node,
                delayed_joint_state_broadcaster,
                delayed_forward_position_controller,
            ])

        # Optional RViz
        if use_rviz:
            node_rviz = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
            )
            nodes_to_launch.append(TimerAction(period=1.0, actions=[node_rviz]))

        # Optional joint_state_publisher_gui
        # if use_joint_state_publisher_gui:
        #     try:
        #         get_package_share_directory("joint_state_publisher_gui")
        #         nodes_to_launch.append(
        #             Node(
        #                 package="joint_state_publisher_gui",
        #                 executable="joint_state_publisher_gui",
        #                 name="joint_state_publisher_gui",
        #             )
        #         )
        #     except PackageNotFoundError:
        #         print(
        #             "[WARNING] joint_state_publisher_gui package not found. Skipping GUI node."
        #         )

        return nodes_to_launch

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

