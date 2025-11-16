from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "tower_crane", package_name="tower_crane_moveit_config"
    ).to_moveit_configs()
    
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="Interface name for can",
        )
    )

    # Include moveit_planning_execution which already includes hardware bringup
    # This includes: hardware bringup, move_group, rviz, and static virtual joint tfs
    moveit_planning_execution = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch" / "moveit_planning_execution.launch.py")
        ),
        launch_arguments={
            "can_interface_name": LaunchConfiguration("can_interface_name"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [moveit_planning_execution])
