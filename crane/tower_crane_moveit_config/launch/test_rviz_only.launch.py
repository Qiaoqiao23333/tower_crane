#!/usr/bin/env python3
"""
RViz test launch file - launches Robot State Publisher + RViz with MoveIt config
Tests if RViz can display the robot model with the MoveIt configuration
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "tower_crane", package_name="tower_crane_moveit_config"
    ).to_moveit_configs()

    # Robot State Publisher - loads and publishes the robot model
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("tower_crane_moveit_config"),
                "launch",
                "rsp.launch.py"
            ])
        )
    )

    # Path to RViz config file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("tower_crane_moveit_config"),
        "config",
        "moveit.rviz"
    ])

    # RViz node with MoveIt config and parameters
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",  # Show output in terminal
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    return LaunchDescription([rsp, rviz_node])

