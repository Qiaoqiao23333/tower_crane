import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

_CANROS_PREFIX = "/home/qiaoqiaochen/appdata/canros/install/tower_crane/share/tower_crane"
_BUS_PATH_PLACEHOLDER = "@BUS_CONFIG_PATH@"


def _prepare_bus_config(share_dir: str) -> str:
    source_path = os.path.join(share_dir, "config", "robot_control", "bus.yml")
    with open(source_path, "r", encoding="utf-8") as infp:
        content = infp.read()

    config_dir = os.path.join(share_dir, "config", "robot_control")
    replacements = [
        (f"{_CANROS_PREFIX}/config/robot_control", config_dir),
        (_BUS_PATH_PLACEHOLDER, config_dir),
    ]

    patched_content = content
    replaced = False
    for needle, target in replacements:
        if needle in patched_content:
            patched_content = patched_content.replace(needle, target)
            replaced = True

    if not replaced:
        raise RuntimeError(
            "Expected CANopen path placeholder not found inside bus.yml; please update "
            "_prepare_bus_config."
        )

    fd, temp_path = tempfile.mkstemp(prefix="tower_crane_bus_", suffix=".yml")
    with os.fdopen(fd, "w", encoding="utf-8") as tmp:
        tmp.write(patched_content)

    return temp_path


def _load_robot_description(
    can_interface: str, bus_config_path: str, master_config_path: str
) -> ParameterValue:
    share_dir = get_package_share_directory("tower_crane")
    urdf_file = os.path.join(share_dir, "urdf", "Tower_crane.urdf")
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    with open(urdf_file, "r", encoding="utf-8") as infp:
        content = infp.read()

    replacements = {
        f"{_CANROS_PREFIX}/config/robot_control/bus.yml": bus_config_path,
        f"{_CANROS_PREFIX}/config/robot_control/master.dcf": master_config_path,
        '<param name="can_interface_name">can0</param>': (
            f'<param name="can_interface_name">{can_interface}</param>'
        ),
    }

    for placeholder, resolved in replacements.items():
        if placeholder not in content:
            raise RuntimeError(
                f"Expected placeholder '{placeholder}' not found in URDF. "
                "Please update the file conversion logic."
            )
        content = content.replace(placeholder, resolved)

    return ParameterValue(content, value_type=str)


def generate_launch_description():
    def launch_setup(context, *args, **kwargs):
        share_dir = get_package_share_directory("tower_crane")
        master_config = os.path.join(share_dir, "config", "robot_control", "master.dcf")
        bus_config_path = _prepare_bus_config(share_dir)
        robot_description = {
            "robot_description": _load_robot_description(
                "vcan0", bus_config_path, master_config
            )
        }
        controller_config = os.path.join(
            share_dir, "config", "tower_crane_ros2_control.yaml"
        )
        rviz_config_file = os.path.join(share_dir, "urdf.rviz")
        slave_config = os.path.join(
            share_dir, "config", "robot_control", "DSY-C.EDS"
        )
        slave_launch = os.path.join(
            get_package_share_directory("canopen_fake_slaves"),
            "launch",
            "cia402_slave.launch.py",
        )

        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controller_config],
            output="screen",
        )

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
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

        node_rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        )

        node_joint_state_publisher_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
        )

        cleanup_action = RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda *_, path=bus_config_path: os.path.exists(path)
                and os.remove(path)
            )
        )

        return [
            node_joint_state_publisher_gui,
            control_node,
            joint_state_broadcaster_spawner,
            forward_position_controller_spawner,
            robot_state_publisher_node,
            slave_node_1,
            slave_node_2,
            slave_node_3,
            node_rviz,
            cleanup_action,
        ]

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
