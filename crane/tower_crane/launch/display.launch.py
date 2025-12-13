import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the path to the package's share directory
    # This is the ROS 2 equivalent of `$(find Tower_crane)`
    pkg_path = get_package_share_directory('tower_crane')

    # Define the path to the URDF file
    urdf_file = os.path.join(pkg_path, 'urdf', 'Tower_crane.urdf')

    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_path, 'urdf.rviz')

    # Read the URDF file content
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # --- Node Definitions ---

    # 1. Joint State Publisher GUI Node
    # This node provides a GUI to control the robot's joints.
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 2. Robot State Publisher Node
    # This node takes the URDF model and publishes the TF transformations between links.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # The robot_description parameter is passed directly to the node
        parameters=[{'robot_description': robot_description_content}]
    )

    # 3. RViz2 Node
    # This node starts RViz2 with the specified configuration file.
    node_rviz = Node(
        package='rviz2',  # Note: The package name is 'rviz2' in ROS 2
        executable='rviz2',
        name='rviz2',
        # The '-d' argument points to the config file
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # --- Launch Description ---
    # Create the launch description and populate it with the nodes
    return LaunchDescription([
        node_joint_state_publisher_gui,
        node_robot_state_publisher,
        node_rviz
    ])