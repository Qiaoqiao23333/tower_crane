from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('tower_crane')

    urdf_file = os.path.join(pkg_path, 'urdf', 'Tower_crane.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    robot_description = {"robot_description": robot_description_content}
    robot_control_config = PathJoinSubstitution(
        [pkg_path, "config", "tower_crane_ros2_control.yaml"]
    )

    rviz_config_file = os.path.join(pkg_path, 'urdf.rviz')

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config],
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
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    slave_config = PathJoinSubstitution(
        [pkg_path, "config/robot_control", "DSY-C.EDS"]
    )

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
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
    
    node_rviz = Node(
        package='rviz2',  # Note: The package name is 'rviz2' in ROS 2
        executable='rviz2',
        name='rviz2',
        # The '-d' argument points to the config file
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    nodes_to_start = [
        node_joint_state_publisher_gui,
        control_node,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
        robot_state_publisher_node,
        slave_node_1,
        slave_node_2,
        slave_node_3,
        node_rviz
    ]

    return LaunchDescription(nodes_to_start)
