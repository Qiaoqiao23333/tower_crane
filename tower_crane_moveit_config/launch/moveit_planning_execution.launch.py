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
            description="Interface name for can (e.g. can0 for real hardware, vcan0 for testing)",
        )
    )

    # Note: To use mock hardware for testing, manually change "hardware_bringup_real.launch.py" 
    # to "hardware_bringup_mock.launch.py" in the hardware_launch_file below

    # Use real hardware by default (hardware_bringup_real.launch.py)
    # Users can override by setting use_mock_hardware:=true
    # For now, we'll default to real hardware - users can manually change the launch file if needed
    hardware_launch_file = PathJoinSubstitution([
        FindPackageShare("tower_crane"),
        "launch",
        "hardware_bringup_real.launch.py"
    ])

    robot_hw_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [hardware_launch_file]
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