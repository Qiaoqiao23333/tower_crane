import os
import re
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

_CANROS_PREFIX = "/home/qiaoqiaochen/appdata/canros/install/tower_crane/share/tower_crane"


def _prepare_bus_config(share_dir: str) -> str:
    source_path = os.path.join(share_dir, "config", "robot_control", "bus.yml")
    with open(source_path, "r", encoding="utf-8") as infp:
        content = infp.read()

    # 如果找到旧占位符，则替换为当前share_dir
    if _CANROS_PREFIX in content:
        patched_content = content.replace(_CANROS_PREFIX, share_dir)
    else:
        # 如果bus.yml中已经是实际路径，确保dcf_path指向正确的路径
        expected_config_path = os.path.join(share_dir, "config", "robot_control")
        # 更新dcf_path为当前正确的路径
        pattern = r'dcf_path:\s*"[^"]*"'
        replacement = f'dcf_path: "{expected_config_path}"'
        patched_content = re.sub(pattern, replacement, content)
    
    # 直接更新bus.yml文件
    with open(source_path, "w", encoding="utf-8") as outfp:
        outfp.write(patched_content)

    return source_path


def _load_robot_description(
    can_interface: str, bus_config_path: str, master_config_path: str
) -> ParameterValue:
    share_dir = get_package_share_directory("tower_crane")
    urdf_file = os.path.join(share_dir, "urdf", "Tower_crane.urdf")
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    with open(urdf_file, "r", encoding="utf-8") as infp:
        content = infp.read()

    # 替换bus_config路径 - 使用正则表达式匹配任何路径
    bus_config_pattern = r'<param name="bus_config">[^<]*</param>'
    bus_config_replacement = f'<param name="bus_config">{bus_config_path}</param>'
    content = re.sub(bus_config_pattern, bus_config_replacement, content)

    # 替换master_config路径 - 使用正则表达式匹配任何路径
    master_config_pattern = r'<param name="master_config">[^<]*</param>'
    master_config_replacement = f'<param name="master_config">{master_config_path}</param>'
    content = re.sub(master_config_pattern, master_config_replacement, content)

    # 替换can_interface_name
    can_interface_pattern = r'<param name="can_interface_name">[^<]*</param>'
    can_interface_replacement = f'<param name="can_interface_name">{can_interface}</param>'
    content = re.sub(can_interface_pattern, can_interface_replacement, content)

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

        # 先启动slave节点和robot_state_publisher，延迟启动control_node
        # 确保slave节点在master尝试连接之前就已经准备好
        nodes_to_launch = [
            robot_state_publisher_node,
            slave_node_1,
            slave_node_2,
            slave_node_3,
        ]
        
        # 延迟3秒后启动control_node，确保slave节点已完全启动并发送Boot-Up帧
        # 同时给master节点足够时间初始化CAN接口
        delayed_control_node = TimerAction(
            period=3.0,
            actions=[control_node],
        )
        
        # 延迟5秒后启动controller spawners，确保control_node已初始化并完成设备连接
        delayed_spawners = TimerAction(
            period=5.0,
            actions=[
                joint_state_broadcaster_spawner,
                forward_position_controller_spawner,
            ],
        )
        
        # 延迟启动rviz，确保系统已初始化
        delayed_rviz = TimerAction(
            period=1.0,
            actions=[node_rviz],
        )

        # Add crane_master nodes to control the motors
        # Using nodes from crane_master package instead of tower_crane package
        
        # Node for hook_joint (node_id: 1)
        crane_master_node_1 = Node(
            package="crane_master",
            executable="tower_crane",
            name="crane_master_hook",
            namespace="hoist",
            parameters=[{
                "can_interface": "vcan0",
                "node_id": "1",
                "gear_ratio": 0.05,
                "auto_start": "true"
            }],
            output="screen",
        )

        # Node for trolley_joint (node_id: 2)
        crane_master_node_2 = Node(
            package="crane_master",
            executable="tower_crane",
            name="crane_master_trolley",
            namespace="trolley",
            parameters=[{
                "can_interface": "vcan0",
                "node_id": "2",
                "gear_ratio": 0.1,
                "auto_start": "true"
            }],
            output="screen",
        )

        # Node for slewing_joint (node_id: 3)
        crane_master_node_3 = Node(
            package="crane_master",
            executable="tower_crane",
            name="crane_master_slewing",
            namespace="slewing",
            parameters=[{
                "can_interface": "vcan0",
                "node_id": "3",
                "gear_ratio": 0.1,
                "auto_start": "true"
            }],
            output="screen",
        )

        # Delay crane_master nodes startup by 5 seconds to ensure CANopen devices are initialized
        delayed_crane_master_nodes = TimerAction(
            period=5.0,
            actions=[
                crane_master_node_1,
                crane_master_node_2,
                crane_master_node_3,
            ],
        )
        
        try:
            get_package_share_directory("joint_state_publisher_gui")
            node_joint_state_publisher_gui = Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            )
            nodes_to_launch.append(node_joint_state_publisher_gui)
        except PackageNotFoundError:
            print("[WARNING] joint_state_publisher_gui package not found. Skipping GUI node.")

        nodes_to_launch.extend([
            delayed_control_node,
            delayed_spawners,
            delayed_rviz,
            delayed_crane_master_nodes,
        ])
        return nodes_to_launch

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
