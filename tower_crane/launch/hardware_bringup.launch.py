import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        "bus.yml"
    ])

    master_config = PathJoinSubstitution([
        FindPackageShare("tower_crane"),
        "config",
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

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controller_config, {
            "bus_config": bus_config,
            "master_config": master_config,
            "master_bin_path": master_bin_path,
        }],
    )

    device_container_node = Node(
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


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    crane_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["crane_controller", "--controller-manager", "/controller_manager"],
    )

    # -------------------------------
    # Return all nodes
    # -------------------------------
    nodes_list = [
        robot_state_publisher_node,
        controller_manager_node,
        device_container_node, 
        joint_state_broadcaster_spawner,
        crane_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_list)
