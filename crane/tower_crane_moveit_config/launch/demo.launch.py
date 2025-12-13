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

    # Include moveit_planning_execution (now supports both mock and real hardware)
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

    return LaunchDescription(declared_arguments + [moveit_planning_execution])
