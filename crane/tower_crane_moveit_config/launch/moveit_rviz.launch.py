from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "tower_crane", package_name="tower_crane_moveit_config"
    ).to_moveit_configs()

    # RViz re-loads MoveGroup parameters at runtime. Removing the trolley joint
    # override here prevents type conflicts when RViz writes the parameter tree.
    joint_limits = moveit_config.joint_limits.get("robot_description_planning", {})
    joint_overrides = joint_limits.get("joint_limits", {})
    joint_overrides.pop("trolley_joint", None)

    # Path to RViz config file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("tower_crane_moveit_config"),
        "config",
        "moveit.rviz"
    ])

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",  # Changed from "log" to "screen" to see RViz output/errors
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([rviz_node])
