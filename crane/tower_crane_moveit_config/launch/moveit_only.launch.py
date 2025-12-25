#!/usr/bin/env python3
"""
MoveIt Only Launch File - For use with separately launched hardware

This launch file ONLY starts MoveIt and RViz, assuming you've already launched:
1. crane_master/canopen_ros2.launch.py (hardware/CANopen communication)
2. ros2 run crane_master moveit_bridge (MoveIt<->CANopen bridge)

Those nodes should already be publishing:
- /robot_description (URDF)
- /joint_states (current joint positions)
- /tf and /tf_static (transforms)

This file launches:
1. Static virtual joint transforms (for MoveIt planning frame)
2. MoveIt move_group node (motion planning)
3. RViz with MoveIt visualization

Usage:
  # After launching crane_master nodes, run:
  ros2 launch tower_crane_moveit_config moveit_only.launch.py

  # Without RViz (headless):
  ros2 launch tower_crane_moveit_config moveit_only.launch.py use_rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start MoveIt RViz (true/false)",
        ),
    ]

    use_rviz = LaunchConfiguration("use_rviz")

    # ============================================================================
    # STEP 1: Static Virtual Joint Transforms
    # ============================================================================
    # These transforms define the planning reference frame for MoveIt
    virtual_joints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tower_crane_moveit_config"),
                    "launch",
                    "static_virtual_joint_tfs.launch.py",
                ]
            )
        ),
    )

    # ============================================================================
    # STEP 2: MoveIt Move Group
    # ============================================================================
    # This is the main MoveIt node that:
    # - Subscribes to /robot_description and /joint_states
    # - Provides motion planning services
    # - Publishes planned trajectories
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tower_crane_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
    )

    # ============================================================================
    # STEP 3: RViz Visualization (Optional)
    # ============================================================================
    # RViz with MoveIt Motion Planning plugin
    # - Subscribes to /robot_description and /joint_states for visualization
    # - Provides interactive planning interface
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tower_crane_moveit_config"),
                    "launch",
                    "moveit_rviz.launch.py",
                ]
            )
        ),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments + [
            virtual_joints,  # 1. Planning frame transforms
            move_group,      # 2. MoveIt motion planning
            rviz,            # 3. Visualization (if use_rviz=true)
        ]
    )


