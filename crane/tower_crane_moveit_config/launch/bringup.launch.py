#!/usr/bin/env python3
"""
Tower Crane Full System Bringup

Alternative launch file that brings up the complete crane system with crane_master nodes.
Use this when you need the CANopen master nodes for motor control.

What it launches:
  1. Robot State Publisher (loads robot model)
  2. Robot Control (ros2_control + optional crane_master nodes)
  3. Static Virtual Joint Transforms
  4. MoveIt Move Group
  5. RViz (optional)

Usage:
  # Full bringup with crane_master and RViz
  ros2 launch tower_crane_moveit_config bringup.launch.py

  # Without crane_master (mock only)
  ros2 launch tower_crane_moveit_config bringup.launch.py use_crane_master:=false

  # With real hardware
  ros2 launch tower_crane_moveit_config bringup.launch.py can_interface_name:=can0

Note: For most use cases, demo.launch.py is recommended as it's simpler.
      Use this launch file when you specifically need crane_master nodes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="CAN interface (e.g. vcan0 for mock, can0 for real)",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="true",
            description="Whether crane_master nodes auto-start the motors (true/false)",
        ),
        DeclareLaunchArgument(
            "use_crane_master",
            default_value="true",
            description="Start crane_master CANopen motor nodes (true/false)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start MoveIt RViz (true/false)",
        ),
    ]

    can_interface_name = LaunchConfiguration("can_interface_name")
    auto_start = LaunchConfiguration("auto_start")
    use_crane_master = LaunchConfiguration("use_crane_master")
    use_rviz = LaunchConfiguration("use_rviz")

    # ============================================================================
    # STEP 1: Robot State Publisher - CRITICAL FIRST STEP
    # ============================================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane_moveit_config"), "launch", "rsp.launch.py"]
            )
        )
    )

    # ============================================================================
    # STEP 2: Robot Control Bringup
    # ============================================================================
    # Launches ros2_control with simulation (fake slaves + CANopen master)
    # Note: crane_master nodes are not included in simulation.launch.py
    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane"), "launch", "simulation.launch.py"]
            )
        ),
        launch_arguments={
            "can_interface_name": can_interface_name,
            "use_rviz": "false",  # MoveIt RViz is launched separately below
            # "use_joint_state_publisher_gui": "false",  # GUI disabled in simulation.launch.py
            "use_robot_state_publisher": "false",  # RSP launched separately via rsp.launch.py
        }.items(),
    )

    # ============================================================================
    # STEP 3: Static Virtual Joint Transforms
    # ============================================================================
    static_virtual_joint_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tower_crane_moveit_config"),
                    "launch",
                    "static_virtual_joint_tfs.launch.py",
                ]
            )
        )
    )

    # ============================================================================
    # STEP 4: MoveIt Move Group
    # ============================================================================
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane_moveit_config"), "launch", "move_group.launch.py"]
            )
        )
    )

    # ============================================================================
    # STEP 5: RViz Visualization (Optional)
    # ============================================================================
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
        condition=IfCondition(use_rviz),
    )

    # ============================================================================
    # Return Launch Description
    # ============================================================================
    return LaunchDescription(
        declared_arguments + [rsp, robot_control, static_virtual_joint_tfs, move_group, moveit_rviz]
    )
