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
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

_CANROS_PREFIX = "/home/qiaoqiaochen/appdata/canros/install/tower_crane/share/tower_crane"
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
    if _BUS_CONFIG_PATH_PLACEHOLDER not in content and _CANROS_PREFIX not in content:
        return source_path

    expected_config_path = os.path.join(share_dir, "config", "robot_control")
    patched_content = content.replace(_BUS_CONFIG_PATH_PLACEHOLDER, expected_config_path)
    patched_content = patched_content.replace(_CANROS_PREFIX, share_dir)

    ros_home = os.environ.get("ROS_HOME", os.path.join(os.path.expanduser("~"), ".ros"))
    out_dir = os.path.join(ros_home, "tower_crane")
    os.makedirs(out_dir, exist_ok=True)

    out_path = os.path.join(out_dir, "bus.patched.yml")
    with open(out_path, "w", encoding="utf-8") as outfp:
        outfp.write(patched_content)

    return out_path


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
        DeclareLaunchArgument(
            "pre_enable_drives",
            default_value="false",
            description=(
                "If true, send a CiA402 enable sequence via cansend (0x6060=1, 0x6040=6/7/15) "
                "to node IDs 1/2/3 before starting controller_manager."
            ),
        ),
        DeclareLaunchArgument(
            "diagnose_canopen",
            default_value="false",
            description=(
                "If true, print CANopen SDO diagnostics (statusword 0x6041 and error code 0x603F) "
                "for node IDs 1/2/3 during bringup."
            ),
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        can_interface_name = LaunchConfiguration("can_interface_name").perform(context)
        share_dir = get_package_share_directory("tower_crane")
        master_config = os.path.join(share_dir, "config", "robot_control", "master.dcf")
        bus_config_path = _prepare_bus_config(share_dir)

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

        pre_enable_step = ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("pre_enable_drives")),
            cmd=[
                "/bin/bash",
                "-lc",
                # Pre-select Profile Position mode and enable drives 1/2/3.
                # We loop and verify the CiA402 statusword (0x6041) indicates Operation Enabled.
                # NOTE: this sends controlwords to real hardware.
                "set -e; CAN_IF='" + can_interface_name + "'; "
                "echo '[pre_enable_drives] enabling CiA402 nodes 1/2/3 on ' ${CAN_IF}; "
                "read_sw(){ local id=$1; "
                "  cansend ${CAN_IF} $(printf '%03X#4041600000000000' $((0x600+id))); "
                "  timeout 0.2s candump -L ${CAN_IF},$(printf '%03X' $((0x580+id))):7FF | tail -n 1 || true; "
                "}; "
                "for id in 1 2 3; do "
                "  echo '[pre_enable_drives] node' ${id} ': set mode 0x6060=1'; "
                "  cansend ${CAN_IF} $(printf '%03X#2F60600001000000' $((0x600+id))); "
                "  for attempt in $(seq 1 30); do "
                "    cansend ${CAN_IF} $(printf '%03X#2B40600006000000' $((0x600+id))); sleep 0.05; "
                "    cansend ${CAN_IF} $(printf '%03X#2B40600007000000' $((0x600+id))); sleep 0.05; "
                "    cansend ${CAN_IF} $(printf '%03X#2B4060000F000000' $((0x600+id))); sleep 0.10; "
                "    sw=$(read_sw ${id}); "
                "    echo '[pre_enable_drives] node' ${id} 'attempt' ${attempt} 'statusword:' ${sw:-<no_reply>}; "
                "    if echo ${sw} | grep -qE '4B416000..27'; then break; fi; "
                "  done; "
                "done",
            ],
            output="screen",
        )

        # Optional diagnostics: poll statusword (0x6041) + error code (0x603F)
        # via SDO and print results. Helps debug enable/activation timeouts.
        canopen_diag = ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("diagnose_canopen")),
            cmd=[
                "/bin/bash",
                "-lc",
                "set -e; CAN_IF='" + can_interface_name + "'; "
                "echo '[canopen_diag] polling statusword (0x6041) + error code (0x603F) for nodes 1/2/3'; "
                "for i in $(seq 1 20); do "
                "for id in 1 2 3; do "
                # statusword 0x6041
                "cansend ${CAN_IF} $(printf '%03X#4041600000000000' $((0x600+id))); "
                "sw=$(timeout 0.2s candump -L ${CAN_IF},$(printf '%03X' $((0x580+id))):7FF | tail -n 1 || true); "
                # error code 0x603F
                "cansend ${CAN_IF} $(printf '%03X#403F600000000000' $((0x600+id))); "
                "ec=$(timeout 0.2s candump -L ${CAN_IF},$(printf '%03X' $((0x580+id))):7FF | tail -n 1 || true); "
                "echo \"[canopen_diag] node ${id} statusword:${sw:-<no_reply>} error_code:${ec:-<no_reply>}\"; "
                "done; "
                "sleep 0.25; "
                "done",
            ],
            output="screen",
        )

        controller_manager_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[robot_description, controller_config],
            name="controller_manager",
        )

        controller_manager_immediate = TimerAction(
            condition=UnlessCondition(LaunchConfiguration("pre_enable_drives")),
            period=0.0,
            actions=[controller_manager_node],
        )

        controller_manager_after_pre_enable = RegisterEventHandler(
            condition=IfCondition(LaunchConfiguration("pre_enable_drives")),
            event_handler=OnProcessExit(
                target_action=pre_enable_step,
                on_exit=[controller_manager_node],
            ),
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

        return [
            robot_state_publisher_node,
            canopen_diag,
            pre_enable_step,
            controller_manager_immediate,
            controller_manager_after_pre_enable,
            joint_state_broadcaster_spawner,
            forward_position_controller_spawner,
        ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])