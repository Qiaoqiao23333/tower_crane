#!/usr/bin/env python3
"""
Minimal RViz test - launches Robot State Publisher + RViz with NO custom config
Use this to test if RViz2 can launch and display the robot model
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    # Minimal RViz node (no custom config, just default view)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription([rsp, rviz_node])

