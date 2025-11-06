from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import TimerAction

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

    robot_hw_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("tower_crane"), "launch", "hardware_bringup.launch.py"])]
        ),
        launch_arguments={
            "can_interface_name": LaunchConfiguration("can_interface_name"),
        }.items(),
    )


    virtual_joints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
            )
        ),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/move_group.launch.py")
        ),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
        ),
    )


    node_list = [
        robot_hw_node,
        virtual_joints,
        move_group,
        rviz,
    ]

    return LaunchDescription(declared_arguments + node_list)