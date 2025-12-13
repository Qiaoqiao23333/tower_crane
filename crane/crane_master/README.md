# crane Master - CANopen ROS 2 Controller

crane Master is a ROS 2-based CANopen motor controller used to control motors that support the CANopen protocol via CAN bus. This controller supports position and velocity control, and provides a series of ROS 2 interfaces for motor monitoring and control.

## Features

- CANopen protocol communication support
- Position and velocity control support
- ROS 2 topics and service interfaces
- Real-time monitoring of motor status, position, and velocity
- Support for motor initialization, start, stop, and reset
- Profile parameter settings support (velocity, acceleration, deceleration)

## Installation

### Prerequisites

- ROS 2 (Humble or higher version recommended)
- CAN interface (e.g., USB-CAN adapter)
- Motors supporting CANopen protocol

### Build and Install

1. Create workspace

   ```bash
   mkdir -p ~/crane_ws/src
   cd ~/crane_ws/src
   ```

2. Clone repository

   ```bash
   git clone https://github.com/Qiaoqiao23333/crane_master.git
   ```

3. Build

   ```bash
   cd ~/crane_ws
   colcon build --packages-select crane_master
   ```

4. Launch node

   ```bash
   source install/setup.bash
   ros2 launch crane_master crane_master.launch.py
   ```

## Configure CAN Interface

### 1. Load CAN modules

```bash
sudo modprobe can
sudo modprobe can_raw
```

### 2. Configure CAN interface (example for can0 with 1Mbps baudrate)

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

### 3. Check CAN interface status

```bash
ip -details link show can0
```

## Usage

### 1. Launch Node

Launch with default parameters

```bash
ros2 launch crane_master canopen_ros2.launch.py
```

### 2. Launch with custom parameters
```bash
# Launch specific node (node_id: 1=hoist, 2=trolley, 3=slewing)
ros2 launch crane_master canopen_ros2.launch.py can_interface:=can0 node_id:=1 auto_start:=true

# Launch all nodes
ros2 launch crane_master canopen_ros2.launch.py can_interface:=can0 node_id:=all
```

## Node ID Mapping

The following mapping is used (matching `bus.yml` configuration):

| Node ID | Motor Name | Namespace | Gear Ratio |
|---------|------------|-----------|------------|
| 1 | hoist_motor | hoist | 20.0 |
| 2 | trolley_motor | trolley | 10.0 |
| 3 | slewing_motor | slewing | 10.0 |

## Controlling the Motor

Each motor has its own namespace, allowing independent control:

### 1. Position Control

Control motor position by publishing to the namespace-specific topic:

- Move hoist to 90 degrees

```bash
ros2 topic pub /hoist/target_position std_msgs/msg/Float32 "data: 90.0" --once
```

- Move trolley to 180 degrees

```bash
ros2 topic pub /trolley/target_position std_msgs/msg/Float32 "data: 180.0" --once
```

- Move slewing to 45 degrees

```bash
ros2 topic pub /slewing/target_position std_msgs/msg/Float32 "data: 45.0" --once
```

### 2. Velocity Control

Control motor velocity by publishing to the namespace-specific topic:

- Set hoist velocity to 10 degrees/second

```bash
ros2 topic pub /hoist/target_velocity std_msgs/msg/Float32 "data: 10.0" --once
```

- Set trolley velocity to -10 degrees/second

```bash
ros2 topic pub /trolley/target_velocity std_msgs/msg/Float32 "data: -10.0" --once
```

## Service Interfaces

Each motor has its own service namespace:

### 1. Start Motor

```bash
# Start hoist motor
ros2 service call /hoist/start_crane std_srvs/srv/Trigger

# Start trolley motor
ros2 service call /trolley/start_crane std_srvs/srv/Trigger

# Start slewing motor
ros2 service call /slewing/start_crane std_srvs/srv/Trigger
```

### 2. Stop Motor

```bash
ros2 service call /hoist/stop_crane std_srvs/srv/Trigger
ros2 service call /trolley/stop_crane std_srvs/srv/Trigger
ros2 service call /slewing/stop_crane std_srvs/srv/Trigger
```

### 3. Reset Motor

```bash
ros2 service call /hoist/reset_crane std_srvs/srv/Trigger
ros2 service call /trolley/reset_crane std_srvs/srv/Trigger
ros2 service call /slewing/reset_crane std_srvs/srv/Trigger
```

## Setting Motor Mode

- Set to position mode

```bash
ros2 service call /hoist/set_crane_mode std_srvs/srv/SetBool "data: true"
```

- Set to velocity mode

```bash
ros2 service call /hoist/set_crane_mode std_srvs/srv/SetBool "data: false"
```

## Monitor Motor Status

```bash
# Monitor hoist motor
ros2 topic echo /hoist/crane_status

# Monitor trolley motor
ros2 topic echo /trolley/crane_status

# Monitor slewing motor
ros2 topic echo /slewing/crane_status
```

## View Motor Position

```bash
ros2 topic echo /hoist/crane_position
ros2 topic echo /trolley/crane_position
ros2 topic echo /slewing/crane_position
```

## View Motor Velocity

```bash
ros2 topic echo /hoist/crane_velocity
ros2 topic echo /trolley/crane_velocity
ros2 topic echo /slewing/crane_velocity
```

## Topic List

All topics are namespace-specific. Replace `{namespace}` with `hoist`, `trolley`, or `slewing`:

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
| /{namespace}/target_position | std_msgs/msg/Float32 | Set target position (degrees) |
| /{namespace}/target_velocity | std_msgs/msg/Float32 | Set target velocity (degrees/second) |
| /{namespace}/crane_status | std_msgs/msg/String | Motor status information |
| /{namespace}/crane_position | std_msgs/msg/Float32 | Current position (degrees) |
| /{namespace}/crane_velocity | std_msgs/msg/Float32 | Current velocity (degrees/second) |

## Service List

All services are namespace-specific. Replace `{namespace}` with `hoist`, `trolley`, or `slewing`:

| Service Name | Service Type | Description |
| ------------ | ------------ | ----------- |
| /{namespace}/start_crane | std_srvs/srv/Trigger | Start motor |
| /{namespace}/stop_crane | std_srvs/srv/Trigger | Stop motor |
| /{namespace}/reset_crane | std_srvs/srv/Trigger | Reset motor |
| /{namespace}/set_crane_mode | std_srvs/srv/SetBool | Set motor mode (true: position mode, false: velocity mode) |

## Parameter List

| Parameter Name | Type | Default Value | Description |
| -------------- | ---- | ------------- | ----------- |
| can_interface | string | "can0" | CAN interface name |
| node_id | string | "1" | CANopen node ID ("1", "2", "3", or "all") |
| gear_ratio | float | 1.0 | Gear ratio (automatically set: hoist=20.0, trolley=10.0, slewing=10.0) |
| auto_start | bool | true | Whether to auto-start motor |
| profile_velocity | int | 5 | Profile velocity (degrees/second) |
| profile_acceleration | int | 5 | Profile acceleration (degrees/second²) |
| profile_deceleration | int | 5 | Profile deceleration (degrees/second²) |

## Troubleshooting

### Cannot Connect to CAN Interface

- Check if CAN interface is properly configured: ip -details link show can0
- Ensure CAN interface is up: sudo ip link set up can0
- Verify CAN bus connections are correct

### Motor Not Responding to Commands

- Check if motor power is connected
- Confirm node ID is correct (1=hoist, 2=trolley, 3=slewing)
- Monitor CAN bus communication using candump: candump can0
- Check motor status: ros2 topic echo /{namespace}/crane_status
- Verify the correct namespace is being used for commands

### Cannot Switch Operation Mode

- Some motors may not support standard CiA402 operation mode switching. In this case, the controller will attempt to work in the current mode.

### Advanced Usage

- Using candump to monitor CAN communication
Install can-utils

```bash
sudo apt-get install can-utils
```

### Monitor CAN bus

```bash
candump can0
```

## License

Apache License 2.0
