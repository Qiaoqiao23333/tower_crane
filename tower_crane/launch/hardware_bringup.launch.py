import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # -------------------------------
    # Launch arguments
    # -------------------------------
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            description="Package where URDF file is stored.",
            default_value="tower_crane"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="CAN interface (e.g. vcan0 or can0)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Use ros2_control with CANopen",
        )
    )

    # -------------------------------
    # File paths
    # -------------------------------
    description_package = LaunchConfiguration("description_package")
    can_interface_name = LaunchConfiguration("can_interface_name")

    controller_config = PathJoinSubstitution([
        FindPackageShare("tower_crane"),
        "config",
        "tower_crane_ros2_control.yaml"
    ])

    bus_config = PathJoinSubstitution([
        FindPackageShare("tower_crane"),
        "config",
        'robot_control',
        "bus.yml"
    ])

    master_config = PathJoinSubstitution([
        FindPackageShare("tower_crane"),
        "config",
        "robot_control",
        "master.dcf"
    ])

    master_bin_path = os.path.join(
        get_package_share_directory("tower_crane"),
        "config",
        "master.bin",
    )
    if not os.path.exists(master_bin_path):
        master_bin_path = ""

    # -------------------------------
    # Read URDF directly (not xacro)
    # -------------------------------
    urdf_path = os.path.join(
        get_package_share_directory("tower_crane"),
        "urdf",
        "Tower_crane.urdf"
    )

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    with open(urdf_path, "r") as infp:
        robot_description_content = infp.read()

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # -------------------------------
    # Nodes
    # -------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Fake CANopen slave devices for testing with vcan0
    # These simulate the actual motors on the CAN bus
    slave_config = PathJoinSubstitution([
        FindPackageShare("tower_crane"),
        "config",
        "robot_control",
        "DSY-C.EDS"
    ])
    
    slave_launch = PathJoinSubstitution([
        FindPackageShare("canopen_fake_slaves"),
        "launch",
        "cia402_slave.launch.py"
    ])
    
    # Node ID 2: slewing_motor
    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "2",
            "node_name": "slave_node_1",
            "slave_config": slave_config,
        }.items(),
    )
    
    # Node ID 3: trolley_motor
    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "3",
            "node_name": "slave_node_2",
            "slave_config": slave_config,
        }.items(),
    )
    
    # Node ID 4: hoist_motor (hook_joint)
    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "4",
            "node_name": "slave_node_3",
            "slave_config": slave_config,
        }.items(),
    )

    # Start device_container_node after fake slaves are ready
    # Give fake slaves time to initialize before master tries to boot them
    device_container_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="canopen_core",
                executable="device_container_node",
                name="device_container_node",
                output="screen",
                parameters=[{
                    "bus_config": bus_config,
                    "master_config": master_config,
                    "master_bin_path": master_bin_path,
                    "can_interface_name": can_interface_name,
                }]
            )
        ]
    )

    # Start controller_manager after device_container is ready
    # The hardware plugin will create its own device container internally
    controller_manager_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[robot_description, controller_config],
                name="controller_manager",
            )
        ]
    )

    # Spawn controllers after controller_manager is ready
    # Give time for the hardware plugin to boot devices and expose services
    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            )
        ]
    )

    forward_position_controller_spawner = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
            )
        ]
    )

    # -------------------------------
    # Return all nodes
    # -------------------------------
    nodes_list = [
        robot_state_publisher_node,
        slave_node_1,  # Start fake slaves first so they're ready when master boots
        slave_node_2,
        slave_node_3,
        device_container_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_list)
