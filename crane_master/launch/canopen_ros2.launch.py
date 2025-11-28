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
    
    # 根据node_id决定启动哪些节点
    if node_id_value == 'all':
        # 启动所有节点 - 为每个节点创建独立的参数字典
        node1_params = {
            'can_interface': can_interface_value,
            'node_id': '1',
            'auto_start': auto_start_value
        }
        node2_params = {
            'can_interface': can_interface_value,
            'node_id': '2',
            'auto_start': auto_start_value
        }
        node3_params = {
            'can_interface': can_interface_value,
            'node_id': '3',
            'auto_start': auto_start_value
        }
        
        nodes_to_launch = [
            Node(
                package='crane_master',
                executable='tower_crane',
                name='canopen_ros2_node1',
                output='screen',
                emulate_tty=True,
                parameters=[node1_params]
            ),
            Node(
                package='crane_master',
                executable='tower_crane',
                name='canopen_ros2_node2',
                output='screen',
                emulate_tty=True,
                parameters=[node2_params]
            ),
            Node(
                package='crane_master',
                executable='tower_crane',
                name='canopen_ros2_node3',
                output='screen',
                emulate_tty=True,
                parameters=[node3_params]
            )
        ]
    elif node_id_value in ['1', '2', '3']:
        # 只启动指定的节点
        nodes_to_launch = [
            Node(
                package='crane_master',
                executable='tower_crane',
                name=f'canopen_ros2_node{node_id_value}',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'can_interface': can_interface_value,
                    'node_id': node_id_value,
                    'auto_start': auto_start_value
                }]
            )
        ]
    else:
        raise ValueError(f"Invalid node_id: {node_id_value}. Must be 'all', '1', '2', or '3'")
    
    return [
        LogInfo(msg=f"启动CANopenROS2节点 (node_id={node_id_value})..."),
        check_can,
        *nodes_to_launch,
        list_info
    ] 