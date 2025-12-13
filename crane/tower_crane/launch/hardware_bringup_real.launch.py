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
import tempfile
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

_CANROS_PREFIX = "/home/qiaoqiaochen/appdata/canros/install/tower_crane/share/tower_crane"
_BUS_CONFIG_PATH_PLACEHOLDER = "@BUS_CONFIG_PATH@"


def _prepare_bus_config(share_dir: str) -> tuple[str, bool]:
    """Return a bus.yml path usable by canopen_core.

    The checked-in bus.yml contains a @BUS_CONFIG_PATH@ placeholder.
    canopen_core expects a real path, so we patch it at runtime.

    Returns (path, should_cleanup).
    """
    source_path = os.path.join(share_dir, "config", "robot_control", "bus.yml")
    with open(source_path, "r", encoding="utf-8") as infp:
        content = infp.read()

    # New-style placeholder: replace @BUS_CONFIG_PATH@ with the expected
    # config directory under this package, and write to a temporary file.
    if _BUS_CONFIG_PATH_PLACEHOLDER in content:
        expected_config_path = os.path.join(share_dir, "config", "robot_control")
        patched_content = content.replace(_BUS_CONFIG_PATH_PLACEHOLDER, expected_config_path)
    # Old-style placeholder from a previous install tree: patch to current
    # share_dir and write to a temporary file.
    elif _CANROS_PREFIX in content:
        patched_content = content.replace(_CANROS_PREFIX, share_dir)
    else:
        # No placeholder: use file in-place and do not delete it on shutdown.
        return source_path, False

    fd, temp_path = tempfile.mkstemp(prefix="tower_crane_bus_", suffix=".yml")
    with os.fdopen(fd, "w", encoding="utf-8") as tmp:
        tmp.write(patched_content)

    return temp_path, True


def _load_robot_description(
    xacro_path: str, can_interface: str, bus_config_path: str, master_config_path: str
) -> ParameterValue:
    """Generate robot_description from xacro so ros2_control uses RobotSystem.

    This ensures the <ros2_control> tag uses canopen_ros2_control/RobotSystem and
    passes bus/master configs + can interface into the hardware plugin.
    """
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"Xacro file not found: {xacro_path}")

    return ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_path,
                " ",
                "can_interface_name:=",
                can_interface,
                " ",
                "bus_config_path:=",
                bus_config_path,
                " ",
                "master_config_path:=",
                master_config_path,
            ]
        ),
        value_type=str,
    )


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            description="Package where URDF file is stored.",
            default_value="tower_crane",
        ),
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="can0",
            description="CAN interface (e.g. can0 for real hardware)",
        ),
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Use ros2_control with CANopen",
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        can_interface_name = LaunchConfiguration("can_interface_name").perform(context)
        share_dir = get_package_share_directory("tower_crane")
        master_config = os.path.join(share_dir, "config", "robot_control", "master.dcf")
        bus_config_path, cleanup_bus_config = _prepare_bus_config(share_dir)

        xacro_path = os.path.join(share_dir, "urdf", "Tower_crane_canopen.urdf.xacro")

        robot_description = {
            "robot_description": _load_robot_description(
                xacro_path, can_interface_name, bus_config_path, master_config
            )
        }
        controller_config = os.path.join(
            share_dir, "config", "tower_crane_ros2_control.yaml"
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )

        controller_manager_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[robot_description, controller_config],
            name="controller_manager",
        )

        joint_state_broadcaster_spawner = TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                )
            ],
        )

        forward_position_controller_spawner = TimerAction(
            period=9.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "forward_position_controller",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                )
            ],
        )

        cleanup_action = RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda *_, path=bus_config_path, cleanup=cleanup_bus_config: cleanup
                and os.path.exists(path)
                and os.remove(path)
            )
        )

        return [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            forward_position_controller_spawner,
            cleanup_action,
        ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])