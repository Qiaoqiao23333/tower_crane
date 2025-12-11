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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    can_interface_arg = DeclareLaunchArgument(
        "can_interface_name",
        default_value="vcan0",
        description="CAN interface (e.g. vcan0 for mock simulation)",
    )
    ld.add_action(can_interface_arg)

    can_interface = LaunchConfiguration("can_interface_name")

    share_dir = get_package_share_directory("tower_crane")
    master_config = os.path.join(
        share_dir,
        "config",
        "robot_control",
        "master.dcf",
    )
    master_bin = os.path.join(
        share_dir,
        "config",
        "robot_control",
        "master.bin",
    )
    bus_config = os.path.join(
        share_dir,
        "config",
        "robot_control",
        "bus.yml",
    )
    slave_config = os.path.join(
        share_dir,
        "config",
        "robot_control",
        "DSY-C.EDS",
    )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "2",
            "node_name": "slave_node_1",
            "slave_config": slave_config,
        }.items(),
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "1",
            "node_name": "slave_node_2",
            "slave_config": slave_config,
        }.items(),
    )

    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "3",
            "node_name": "slave_node_3",
            "slave_config": slave_config,
        }.items(),
    )

    # Add slave nodes first to ensure they are ready before master tries to connect
    ld.add_action(slave_node_1)
    ld.add_action(slave_node_2)
    ld.add_action(slave_node_3)

    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": master_config,
            "master_bin": master_bin if os.path.exists(master_bin) else "",
            "bus_config": bus_config,
            "can_interface_name": can_interface,
            "master.can_interface_name": can_interface,
        }.items(),
    )
    
    # Delay device_container startup by 0.1 seconds to ensure fake slaves are ready
    delayed_device_container = TimerAction(
        period=0.1,
        actions=[device_container],
    )
    
    ld.add_action(delayed_device_container)

    # NOTE: position_tick_motor_node (a legacy test client that calls /<joint>/{init,cyclic_position_mode,target}
    # services from canopen_ros2_control) is no longer auto-launched here. The canonical way to command the
    # three CANopen motors in both real and mock setups is via ros2_control controllers (e.g. through MoveIt
    # using the forward_position_controller FollowJointTrajectory action), so we keep the node only as an
    # optional test tool that can be run manually if needed.

    return ld
