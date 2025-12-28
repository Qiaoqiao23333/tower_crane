#!/usr/bin/env python3
"""
MoveIt Planning and Execution Launch File

This launch file brings up the complete tower crane system with MoveIt motion planning:
  1. Robot State Publisher (loads robot model)
  2. Hardware interface (mock or real CANopen)
  3. Static transforms for virtual joints
  4. MoveIt move_group node
  5. RViz visualization (optional)

Usage:
  # Mock hardware (simulation)
  ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py

  # Real hardware (auto-detected from can0)
  ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py \
      can_interface_name:=can0

  # Without RViz
  ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py use_rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    """Setup function to evaluate launch arguments at runtime."""
    
    # Get the CAN interface name
    can_interface_name_str = LaunchConfiguration("can_interface_name").perform(context)
    
    # Auto-detect if we should use real hardware based on CAN interface name
    # can0, can1, etc. = real hardware
    # vcan0, vcan1, etc. = virtual/mock hardware
    use_real_hardware = can_interface_name_str.startswith("can") and not can_interface_name_str.startswith("vcan")
    
    can_interface_name = LaunchConfiguration("can_interface_name")
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
    # STEP 2: Hardware/Control Bringup
    # ============================================================================
    
    if use_real_hardware:
        # Real hardware path (CANopen RobotSystem)
        hardware = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("tower_crane"), "launch", "hardware_bringup_real.launch.py"]
                )
            ),
            launch_arguments={
                "can_interface_name": can_interface_name,    # Enable CANopen diagnostics
            }.items(),
        )
    else:
        # Mock hardware path (for simulation/testing)
        hardware = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("tower_crane"), "launch", "simulation.launch.py"]
                )
            ),
            launch_arguments={
                "can_interface_name": can_interface_name,
                "use_rviz": "false",  # RViz launched separately below
                # "use_joint_state_publisher_gui": "false",  # GUI disabled in simulation.launch.py
                "use_robot_state_publisher": "false",  # RSP launched separately via rsp.launch.py
            }.items(),
        )

    # ============================================================================
    # STEP 3: Static Virtual Joint Transforms
    # ============================================================================
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
    # STEP 4: MoveIt Move Group
    # ============================================================================
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane_moveit_config"), "launch", "move_group.launch.py"]
            )
        ),
    )

    # ============================================================================
    # STEP 5: RViz Visualization (Optional)
    # ============================================================================
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
        condition=IfCondition(use_rviz),
    )

    return [
        rsp,              # 1. Robot model
        hardware,         # 2. Hardware (auto-selected based on CAN interface)
        virtual_joints,   # 3. Static transforms
        move_group,       # 4. Motion planning
        rviz,             # 5. Visualization (if use_rviz=true)
    ]


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="CAN interface (vcan0 for mock, can0/can1/... for real hardware). Hardware type is auto-detected.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start MoveIt RViz (true/false)",
        ),
    ]

    # Use OpaqueFunction to evaluate arguments at runtime
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
