import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def _load_robot_description() -> ParameterValue:
    share_dir = get_package_share_directory("tower_crane")
    urdf_file = os.path.join(share_dir, "urdf", "Tower_crane.urdf")
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    with open(urdf_file, "r", encoding="utf-8") as infp:
        return ParameterValue(infp.read(), value_type=str)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="CAN interface to use (vcan0 for mock, can0 for real)",
        ),
        DeclareLaunchArgument(
            "use_real_hardware",
            default_value="false",
            description=(
                "If true, bring up real hardware via ros2_canopen/canopen_ros2_control (hardware_bringup_real). "
                "If false, run the mock setup (fake slaves + mock ros2_control)."
            ),
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="true",
            description="Whether crane_master nodes auto-start the motors (true/false) [mock mode only]",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start the tower_crane URDF RViz config (true/false)",
        ),
        DeclareLaunchArgument(
            "use_crane_master",
            default_value="true",
            description="Start crane_master SocketCAN motor nodes (true/false) [mock mode only]",
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        can_interface_name = LaunchConfiguration("can_interface_name").perform(context)
        use_real_hardware = (
            LaunchConfiguration("use_real_hardware").perform(context).lower() == "true"
        )
        auto_start = LaunchConfiguration("auto_start").perform(context)
        use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() == "true"
        use_crane_master = (
            LaunchConfiguration("use_crane_master").perform(context).lower() == "true"
        )

        share_dir = get_package_share_directory("tower_crane")
        rviz_config_file = os.path.join(share_dir, "urdf.rviz")

        # Real hardware path: use the standard ros2_canopen + canopen_ros2_control stack.
        # This will start controller_manager + CANopen drivers using bus.yml/master.dcf.
        if use_real_hardware:
            hardware_real_launch = os.path.join(
                share_dir, "launch", "hardware_bringup_real.launch.py"
            )
            hardware_real = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hardware_real_launch),
                launch_arguments={
                    "can_interface_name": can_interface_name,
                }.items(),
            )

            nodes_to_launch = [hardware_real]

            if use_rviz:
                node_rviz = Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    arguments=["-d", rviz_config_file],
                    output="screen",
                )
                nodes_to_launch.append(TimerAction(period=1.0, actions=[node_rviz]))

            try:
                get_package_share_directory("joint_state_publisher_gui")
                nodes_to_launch.append(
                    Node(
                        package="joint_state_publisher_gui",
                        executable="joint_state_publisher_gui",
                        name="joint_state_publisher_gui",
                    )
                )
            except PackageNotFoundError:
                print(
                    "[WARNING] joint_state_publisher_gui package not found. Skipping GUI node."
                )

            return nodes_to_launch

        # Mock path below (fake slaves + mock ros2_control + optional crane_master test nodes)
        robot_description = {"robot_description": _load_robot_description()}

        controller_config = os.path.join(share_dir, "config", "tower_crane_ros2_control.yaml")
        slave_config = os.path.join(share_dir, "config", "robot_control", "DSY-C.EDS")
        slave_launch = os.path.join(
            get_package_share_directory("canopen_fake_slaves"),
            "launch",
            "cia402_slave.launch.py",
        )

        # NOTE: Controllers (e.g. JointTrajectoryController) are created as additional nodes
        # inside the ros2_control_node process. To make sure their per-controller parameters
        # (e.g. forward_position_controller.ros__parameters.joints/command_interfaces) are
        # available, pass the YAML as a *global* params file argument.
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description],
            arguments=["--ros-args", "--params-file", controller_config],
            output="screen",
        )

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

        forward_position_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "forward_position_controller",
                "--controller-manager",
                "/controller_manager",
            ],
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )

        slave_node_1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slave_launch),
            launch_arguments={
                "node_id": "2",
                "node_name": "slave_node_1",
                "slave_config": slave_config,
            }.items(),
        )

        slave_node_2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slave_launch),
            launch_arguments={
                "node_id": "1",
                "node_name": "slave_node_2",
                "slave_config": slave_config,
            }.items(),
        )

        slave_node_3 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slave_launch),
            launch_arguments={
                "node_id": "3",
                "node_name": "slave_node_3",
                "slave_config": slave_config,
            }.items(),
        )

        nodes_to_launch = [
            robot_state_publisher_node,
            slave_node_1,
            slave_node_2,
            slave_node_3,
        ]

        delayed_control_node = TimerAction(period=3.0, actions=[control_node])
        delayed_spawners = TimerAction(
            period=5.0,
            actions=[
                joint_state_broadcaster_spawner,
                forward_position_controller_spawner,
            ],
        )

        nodes_to_launch.extend([delayed_control_node, delayed_spawners])

        if use_rviz:
            node_rviz = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
            )
            nodes_to_launch.append(TimerAction(period=1.0, actions=[node_rviz]))

        # NOTE: crane_master uses a custom SocketCAN-based motor driver and should not
        # be launched together with the ros2_canopen stack (real hardware mode).
        if use_crane_master:
            crane_master_node_1 = Node(
                package="crane_master",
                executable="tower_crane",
                name="crane_master_hook",
                namespace="hoist",
                parameters=[
                    {
                        "can_interface": can_interface_name,
                        "node_id": "1",
                        "gear_ratio": 0.05,
                        "auto_start": auto_start,
                    }
                ],
                output="screen",
            )

            crane_master_node_2 = Node(
                package="crane_master",
                executable="tower_crane",
                name="crane_master_trolley",
                namespace="trolley",
                parameters=[
                    {
                        "can_interface": can_interface_name,
                        "node_id": "2",
                        "gear_ratio": 0.1,
                        "auto_start": auto_start,
                    }
                ],
                output="screen",
            )

            crane_master_node_3 = Node(
                package="crane_master",
                executable="tower_crane",
                name="crane_master_slewing",
                namespace="slewing",
                parameters=[
                    {
                        "can_interface": can_interface_name,
                        "node_id": "3",
                        "gear_ratio": 0.1,
                        "auto_start": auto_start,
                    }
                ],
                output="screen",
            )

            nodes_to_launch.append(
                TimerAction(
                    period=5.0,
                    actions=[
                        crane_master_node_1,
                        crane_master_node_2,
                        crane_master_node_3,
                    ],
                )
            )

        try:
            get_package_share_directory("joint_state_publisher_gui")
            nodes_to_launch.append(
                Node(
                    package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    name="joint_state_publisher_gui",
                )
            )
        except PackageNotFoundError:
            print("[WARNING] joint_state_publisher_gui package not found. Skipping GUI node.")

        return nodes_to_launch

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
