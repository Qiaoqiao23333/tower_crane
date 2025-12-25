from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("tower_crane", package_name="tower_crane_moveit_config").to_moveit_configs()
    
    # Get xacro file path
    xacro_file = PathJoinSubstitution([
        FindPackageShare("tower_crane_moveit_config"),
        "config",
        "tower_crane.urdf.xacro"
    ])
    
    # Process xacro file (it will automatically resolve the base URDF path)
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ]
    )
    
    # Create robot state publisher node with processed URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description_content, value_type=str)
            }
        ],
    )
    
    return LaunchDescription([robot_state_publisher])
