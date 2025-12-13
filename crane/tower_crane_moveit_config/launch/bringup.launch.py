from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="CAN interface (e.g. vcan0 for mock, can0 for real)",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="true",
            description="Whether crane_master nodes auto-start the motors (true/false)",
        ),
        DeclareLaunchArgument(
            "use_crane_master",
            default_value="true",
            description="Start crane_master CANopen motor nodes (true/false)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start MoveIt RViz (true/false)",
        ),
    ]

    can_interface_name = LaunchConfiguration("can_interface_name")
    auto_start = LaunchConfiguration("auto_start")
    use_crane_master = LaunchConfiguration("use_crane_master")
    use_rviz = LaunchConfiguration("use_rviz")

    # 1) Robot/control bringup (ros2_control + fake slaves + optional crane_master nodes)
    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane"), "launch", "robot_control.launch.py"]
            )
        ),
        launch_arguments={
            "can_interface_name": can_interface_name,
            "auto_start": auto_start,
            "use_crane_master": use_crane_master,
            "use_rviz": "false",  # MoveIt RViz is launched separately below
        }.items(),
    )

    # 2) MoveIt
    static_virtual_joint_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tower_crane_moveit_config"),
                    "launch",
                    "static_virtual_joint_tfs.launch.py",
                ]
            )
        )
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane_moveit_config"), "launch", "move_group.launch.py"]
            )
        )
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tower_crane_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments + [robot_control, static_virtual_joint_tfs, move_group, moveit_rviz]
    )
