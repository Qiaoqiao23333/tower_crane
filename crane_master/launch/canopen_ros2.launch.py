from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN接口名称'
        ),
        DeclareLaunchArgument(
            'node_id',
            default_value='all',
            description='CANopen节点ID (all, 1, 2, or 3). Default: all (launch all nodes)'
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='是否自动启动电机'
        ),
        
        # 使用OpaqueFunction来根据node_id动态创建节点
        OpaqueFunction(function=launch_nodes)
    ])

def launch_nodes(context):
    # 获取node_id的实际值
    node_id_value = context.launch_configurations.get('node_id', 'all')
    can_interface_value = context.launch_configurations.get('can_interface', 'can0')
    auto_start_value = context.launch_configurations.get('auto_start', 'true')
    
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
    
    nodes_to_launch = []
    
    # 定义各节点配置（与bus.yml中的映射一致）
    # Node 1 (起升/Hoist): namespace=hoist, gear_ratio=20.0
    node1_config = {
        'package': 'crane_master',
        'executable': 'tower_crane',
        'name': 'canopen_ros2_node1',
        'namespace': 'hoist',
        'parameters': [{
            'can_interface': can_interface_value,
            'node_id': '1',
            'gear_ratio': 0.05,
            'auto_start': auto_start_value
        }]
    }

    # Node 2 (小车/Trolley): namespace=trolley, gear_ratio=10.0
    node2_config = {
        'package': 'crane_master',
        'executable': 'tower_crane',
        'name': 'canopen_ros2_node2',
        'namespace': 'trolley',
        'parameters': [{
            'can_interface': can_interface_value,
            'node_id': '2',
            'gear_ratio': 0.1,
            'auto_start': auto_start_value
        }]
    }

    # Node 3 (回转/Slewing): namespace=slewing, gear_ratio=10.0
    node3_config = {
        'package': 'crane_master',
        'executable': 'tower_crane',
        'name': 'canopen_ros2_node3',
        'namespace': 'slewing',
        'parameters': [{
            'can_interface': can_interface_value,
            'node_id': '3',
            'gear_ratio': 0.1,
            'auto_start': auto_start_value
        }]
    }
    
    all_configs = {
        '1': node1_config,
        '2': node2_config,
        '3': node3_config
    }

    # 根据node_id决定启动哪些节点
    if node_id_value == 'all':
        # 启动所有节点
        for nid in ['1', '2', '3']:
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
    elif node_id_value in ['1', '2', '3']:
        # 只启动指定的节点
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
        raise ValueError(f"Invalid node_id: {node_id_value}. Must be 'all', '1', '2', or '3'")
    
    return [
        LogInfo(msg=f"启动CANopenROS2节点 (node_id={node_id_value})..."),
        check_can,
        *nodes_to_launch,
        list_info
    ]