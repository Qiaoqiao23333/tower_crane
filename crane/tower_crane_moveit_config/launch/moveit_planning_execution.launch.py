from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "tower_crane", package_name="tower_crane_moveit_config"
    ).to_moveit_configs()

    declared_arguments = [
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="CAN interface (vcan0 for mock, can0 for real hardware)",
        ),
        DeclareLaunchArgument(
            "use_real_hardware",
            default_value="false",
            description="If true, bring up CANopen RobotSystem on can_interface_name (real hardware). If false, use mock bringup.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start MoveIt RViz (true/false)",
        ),
    ]

    can_interface_name = LaunchConfiguration("can_interface_name")
    use_real_hardware = LaunchConfiguration("use_real_hardware")
    use_rviz = LaunchConfiguration("use_rviz")

    # 1) Hardware/control bringup
    # - mock path: tower_crane/launch/robot_control.launch.py (loads controllers reliably)
    # - real path: tower_crane/launch/hardware_bringup_real.launch.py (RobotSystem)
    hardware_mock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane"), "launch", "robot_control.launch.py"]
            )
        ),
        condition=UnlessCondition(use_real_hardware),
        launch_arguments={
            "can_interface_name": can_interface_name,
            "auto_start": "true",
            "use_rviz": "false",
            "use_crane_master": "false",
        }.items(),
    )

    hardware_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane"), "launch", "hardware_bringup_real.launch.py"]
            )
        ),
        condition=IfCondition(use_real_hardware),
        launch_arguments={
            "can_interface_name": can_interface_name,
        }.items(),
    )

    # 2) MoveIt
    virtual_joints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py")
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
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            hardware_mock,
            hardware_real,
            virtual_joints,
            move_group,
            rviz,
        ]
    )
