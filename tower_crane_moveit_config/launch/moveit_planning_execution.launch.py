from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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
            default_value="can0",
            description="Interface name for CAN (e.g. can0 for real hardware, vcan0 for testing)",
        )
    )

    can_interface_name = LaunchConfiguration("can_interface_name")

    # Hardware bringup (real hardware only; mock handled by separate launch if needed)
    hardware_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane"), "launch", "hardware_bringup_real.launch.py"]
            )
        ),
        launch_arguments={
            "can_interface_name": can_interface_name,
        }.items(),
    )


    # Static virtual joint TFs for MoveIt
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
        hardware_real,
        virtual_joints,
        move_group,
        rviz,
    ]

    return LaunchDescription(declared_arguments + node_list)