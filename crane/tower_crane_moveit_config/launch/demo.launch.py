#!/usr/bin/env python3
"""
Tower Crane MoveIt Demo Launch File

This is the MAIN LAUNCH FILE for the tower crane with MoveIt motion planning.
It provides a unified interface for both mock (simulation) and real hardware.

What it launches:
  1. Robot State Publisher (loads robot model)
  2. MoveIt Planning & Execution (hardware + motion planning + RViz)

Usage:
  # Basic demo with mock hardware and RViz (RECOMMENDED FOR TESTING)
  ros2 launch tower_crane_moveit_config demo.launch.py

  # Real hardware with RViz
  ros2 launch tower_crane_moveit_config demo.launch.py \
      use_real_hardware:=true can_interface_name:=can0

  # Mock hardware without RViz (headless)
  ros2 launch tower_crane_moveit_config demo.launch.py use_rviz:=false

Expected result:
  - RViz opens with tower crane robot model visible
  - MoveIt Motion Planning panel available
  - Ready to plan and execute motions!
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "tower_crane", package_name="tower_crane_moveit_config"
    ).to_moveit_configs()
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="CAN interface (vcan0 for mock, can0 for real hardware)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_real_hardware",
            default_value="false",
            description="If true, use CANopen RobotSystem on can_interface_name; if false, use mock bringup.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start MoveIt RViz (true/false)",
        )
    )
    
    # Robot State Publisher - loads robot URDF and publishes TF tree
    # This is the foundation - must launch first!
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("tower_crane_moveit_config"),
                "launch",
                "rsp.launch.py"
            ])
        )
    )

    # MoveIt Planning & Execution - includes hardware, move_group, and RViz
    # This handles everything else (hardware control + motion planning + visualization)
    moveit_planning_execution = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch" / "moveit_planning_execution.launch.py")
        ),
        launch_arguments={
            "can_interface_name": LaunchConfiguration("can_interface_name"),
            "use_real_hardware": LaunchConfiguration("use_real_hardware"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [rsp, moveit_planning_execution])
