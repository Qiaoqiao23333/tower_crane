#!/usr/bin/env python3
"""
Tower Crane MoveIt Bringup — single entry-point for every launch scenario.

Scenarios
---------
1. Simulation (mock hardware) + MoveIt + RViz  [DEFAULT]
   ros2 launch tower_crane_moveit_config bringup.launch.py

2. Real hardware + MoveIt + RViz
   ros2 launch tower_crane_moveit_config bringup.launch.py can_interface_name:=can0

3. MoveIt-only (hardware already running via crane_master + moveit_bridge)
   ros2 launch tower_crane_moveit_config bringup.launch.py launch_hardware:=false

4. Headless (no RViz)
   ros2 launch tower_crane_moveit_config bringup.launch.py use_rviz:=false

Hardware type (mock vs real) is auto-detected from the CAN interface name:
  vcan* → mock / simulation  (uses ros2_control with mock hardware)
  can*  → real CANopen hardware (uses crane_master + moveit_bridge)
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_static_virtual_joint_tfs_launch,
)


# ---------------------------------------------------------------------------
# Runtime setup (evaluated when the launch system resolves arguments)
# ---------------------------------------------------------------------------
def _launch_setup(context, *args, **kwargs):
    can_interface = LaunchConfiguration("can_interface_name").perform(context)
    launch_hw = LaunchConfiguration("launch_hardware").perform(context).lower() == "true"
    rviz_enabled = LaunchConfiguration("use_rviz").perform(context).lower() == "true"

    # Auto-detect real vs mock hardware
    use_real_hardware = (
        can_interface.startswith("can") and not can_interface.startswith("vcan")
    )

    # Build MoveIt configuration once
    moveit_config = MoveItConfigsBuilder(
        "tower_crane", package_name="tower_crane_moveit_config"
    ).to_moveit_configs()

    entities = []

    # ==================================================================
    # 1. Robot State Publisher  (only when we also launch hardware)
    # ==================================================================
    if launch_hw:
        xacro_file = PathJoinSubstitution([
            FindPackageShare("tower_crane_moveit_config"),
            "config",
            "tower_crane.urdf.xacro",
        ])
        robot_description_content = Command([
            FindExecutable(name="xacro"), " ", xacro_file,
        ])
        entities.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[{
                    "robot_description": ParameterValue(
                        robot_description_content, value_type=str
                    ),
                }],
            )
        )

    # ==================================================================
    # 2. Hardware / Control bringup  (only when we also launch hardware)
    # ==================================================================
    if launch_hw:
        if use_real_hardware:
            # Launch crane_master CANopen nodes (hoist, trolley, slewing)
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("crane_master"),
                            "launch",
                            "canopen_ros2.launch.py",
                        ])
                    ),
                    launch_arguments={
                        "can_interface": LaunchConfiguration("can_interface_name"),
                    }.items(),
                )
            )
            # Launch moveit_bridge (translates MoveIt trajectories → motor commands)
            entities.append(
                Node(
                    package="crane_master",
                    executable="moveit_bridge",
                    name="moveit_bridge",
                    output="screen",
                )
            )
        else:
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("tower_crane"),
                            "launch",
                            "simulation.launch.py",
                        ])
                    ),
                    launch_arguments={
                        "can_interface_name": LaunchConfiguration("can_interface_name"),
                        "use_rviz": "false",
                        "use_robot_state_publisher": "false",
                    }.items(),
                )
            )

    # ==================================================================
    # 3. Static virtual-joint transforms
    # ==================================================================
    entities.extend(
        generate_static_virtual_joint_tfs_launch(moveit_config).entities
    )

    # ==================================================================
    # 4. MoveIt move_group
    # ==================================================================
    entities.extend(
        generate_move_group_launch(moveit_config).entities
    )

    # ==================================================================
    # 5. RViz with MoveIt Motion Planning panel  (optional)
    # ==================================================================
    if rviz_enabled:
        # Avoid parameter-type conflict when RViz re-writes the param tree
        joint_limits = moveit_config.joint_limits.get(
            "robot_description_planning", {}
        )
        joint_limits.get("joint_limits", {}).pop("trolley_joint", None)

        rviz_config = PathJoinSubstitution([
            FindPackageShare("tower_crane_moveit_config"),
            "config",
            "moveit_v2.rviz",
        ])
        entities.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
            )
        )

    return entities


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="can0",
            description=(
                "CAN interface name. "
                "vcan* → mock/simulation hardware, can* → real CANopen hardware."
            ),
        ),
        DeclareLaunchArgument(
            "launch_hardware",
            default_value="true",
            description=(
                "Launch the hardware stack. "
                "For real hardware: launches crane_master + moveit_bridge. "
                "For mock: launches ros2_control simulation. "
                "Set to false when hardware nodes are already running."
            ),
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with MoveIt Motion Planning panel.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
