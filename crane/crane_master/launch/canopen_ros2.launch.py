import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def load_crane_params():
    """Load crane parameters from crane_params.yaml."""
    config_path = os.path.join(
        get_package_share_directory('crane_master'), 'config', 'crane_params.yaml'
    )
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def merge_params(common, node_section):
    """Merge common defaults with per-node overrides (node overrides win)."""
    merged = dict(common)
    merged.update(node_section)
    return merged


def generate_launch_description():
    return LaunchDescription([
        # 声明参数 (launch-arg overrides, empty means use YAML value)
        DeclareLaunchArgument(
            'can_interface_name',
            default_value='can0',
            description='CAN接口名称 (override YAML value)'
        ),
        DeclareLaunchArgument(
            'node_id',
            default_value='all',
            description='CANopen节点ID (all, 1, 2, or 3). Default: all (launch all nodes)'
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='',
            description='是否自动启动电机 (override YAML value)'
        ),

        # 使用OpaqueFunction来根据node_id动态创建节点
        OpaqueFunction(function=launch_nodes)
    ])


def launch_nodes(context):
    # 加载YAML配置
    params = load_crane_params()
    common = params.get('common', {})

    # Launch-arg overrides (non-empty values take precedence over YAML)
    can_interface_override = context.launch_configurations.get('can_interface_name', '')
    auto_start_override = context.launch_configurations.get('auto_start', '')
    node_id_value = context.launch_configurations.get('node_id', 'all')

    if can_interface_override:
        common['can_interface_name'] = can_interface_override
    if auto_start_override:
        common['auto_start'] = auto_start_override.lower() in ('true', '1', 'yes')

    can_interface_value = common.get('can_interface_name', 'vcan0')

    # 检查CAN接口状态
    check_can = ExecuteProcess(
        cmd=['bash', '-c', f'ip -details link show {can_interface_value} || echo "CAN接口不存在"'],
        output='screen'
    )

    # 列出节点、话题和服务
    list_info = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 5 && echo "列出所有节点:" && ros2 node list && echo "列出所有话题:" && ros2 topic list && echo "列出所有服务:" && ros2 service list'],
        output='screen'
    )

    # 定义各节点配置（从YAML加载 + common合并）
    axis_defs = {
        'hoist':   {'name': 'canopen_ros2_node1', 'namespace': 'hoist'},
        'trolley': {'name': 'canopen_ros2_node2', 'namespace': 'trolley'},
        'slewing': {'name': 'canopen_ros2_node3', 'namespace': 'slewing'},
    }

    # node_id → axis key 映射
    nid_to_axis = {}
    for axis_key in axis_defs:
        section = params.get(axis_key, {})
        nid = str(section.get('node_id', ''))
        nid_to_axis[nid] = axis_key

    all_configs = {}
    for axis_key, meta in axis_defs.items():
        section = params.get(axis_key, {})
        merged = merge_params(common, section)
        nid = str(merged['node_id'])
        all_configs[nid] = {
            'package': 'crane_master',
            'executable': 'tower_crane',
            'name': meta['name'],
            'namespace': meta['namespace'],
            'parameters': [{
                'can_interface_name':        str(merged['can_interface_name']),
                'node_id':             str(merged['node_id']),
                'gear_ratio':          float(merged['gear_ratio']),
                'target_units_per_rev': int(merged.get('target_units_per_rev', 10000)),
                'auto_start':          bool(merged.get('auto_start', True)),
                'profile_velocity':    float(merged.get('profile_velocity', 30.0)),
                'profile_acceleration': float(merged.get('profile_acceleration', 30.0)),
                'profile_deceleration': float(merged.get('profile_deceleration', 30.0)),
                'cycle_period_us':     int(merged.get('cycle_period_us', 1000)),
            }]
        }

    nodes_to_launch = []

    # 根据node_id决定启动哪些节点
    if node_id_value == 'all':
        for nid in sorted(all_configs.keys()):
            cfg = all_configs[nid]
            nodes_to_launch.append(
                Node(
                    package=cfg['package'],
                    executable=cfg['executable'],
                    name=cfg['name'],
                    namespace=cfg['namespace'],
                    output='screen',
                    emulate_tty=True,
                    parameters=cfg['parameters']
                )
            )
    elif node_id_value in all_configs:
        cfg = all_configs[node_id_value]
        nodes_to_launch.append(
            Node(
                package=cfg['package'],
                executable=cfg['executable'],
                name=cfg['name'],
                namespace=cfg['namespace'],
                output='screen',
                emulate_tty=True,
                parameters=cfg['parameters']
            )
        )
    else:
        valid = list(all_configs.keys()) + ['all']
        raise ValueError(f"Invalid node_id: {node_id_value}. Must be one of {valid}")

    return [
        LogInfo(msg=f"启动CANopenROS2节点 (node_id={node_id_value})..."),
        check_can,
        *nodes_to_launch,
        list_info
    ]
