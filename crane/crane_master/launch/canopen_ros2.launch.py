from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='📡 CAN interface name'
        ),
        DeclareLaunchArgument(
            'node_id',
            default_value='all',
            description='🆔 CANopen node ID (all, 1, 2, or 3), default: all (start all nodes)'
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='⚡ Whether to auto-start motors'
        ),
        
        # Use OpaqueFunction to dynamically create nodes based on node_id
        OpaqueFunction(function=launch_nodes)
    ])

def launch_nodes(context):
    # Get actual values of node_id
    node_id_value = context.launch_configurations.get('node_id', 'all')
    can_interface_value = context.launch_configurations.get('can_interface', 'can0')
    auto_start_value = context.launch_configurations.get('auto_start', 'true')
    
    # Check CAN interface status
    check_can = ExecuteProcess(
        cmd=['bash', '-c', f'ip -details link show {can_interface_value} || echo "CAN interface does not exist"'],
        output='screen'
    )
    
    # List nodes, topics and services
    list_info = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 5 && echo "List all nodes:" && ros2 node list && echo "List all topics:" && ros2 topic list && echo "List all services:" && ros2 service list'],
        output='screen'
    )
    
    nodes_to_launch = []
    
    # Define node configurations (consistent with mapping in bus.yml)
    # Node 1 (Hoist): namespace=hoist, gear_ratio=20.0
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

    # Node 2 (Trolley): namespace=trolley, gear_ratio=10.0
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

    # Node 3 (Slewing): namespace=slewing, gear_ratio=10.0
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

    # Determine which nodes to launch based on node_id
    if node_id_value == 'all':
        # Launch all nodes
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
        # Launch only the specified node
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
    
    # Add synchronized trajectory action server
    sync_trajectory_server = Node(
        package='crane_master',
        executable='sync_trajectory_action_server',
        name='sync_trajectory_action_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'can_interface': can_interface_value
        }]
    )
    
    return [
        LogInfo(msg=f"🏗️ Starting CANopen ROS2 nodes (node_id={node_id_value})..."),
        check_can,
        *nodes_to_launch,
        sync_trajectory_server,
        list_info
    ]