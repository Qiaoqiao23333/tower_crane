# crane Master - CANopen ROS 2 Controller

crane Master is a ROS 2-based CANopen motor controller used to control motors that support the CANopen protocol via CAN bus. This controller supports position, velocity, and CiA402 homing (HM) control, and provides a series of ROS 2 interfaces for motor monitoring and control.

## Features

- CANopen protocol communication support
- Position and velocity control support
- CiA402 homing mode (HM) support
- ROS 2 topics and service interfaces
- Real-time monitoring of motor status, position, and velocity
- Support for motor initialization, start, stop, and reset
- Profile parameter settings support (velocity, acceleration, deceleration)

## Installation

### Prerequisites

- ROS 2 (Humble or higher version recommended)
- CAN interface (e.g., USB-CAN adapter)
- Motors supporting CANopen protocol

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
ros2 launch crane_master canopen_ros2.launch.py can_interface_name:=can0 node_id:=1 auto_start:=true

# Launch all nodes
ros2 launch crane_master canopen_ros2.launch.py can_interface_name:=can0 node_id:=all
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

### 4. Home Motor (CiA402 HM)

```bash
ros2 service call /hoist/home_crane std_srvs/srv/Trigger
ros2 service call /trolley/home_crane std_srvs/srv/Trigger
ros2 service call /slewing/home_crane std_srvs/srv/Trigger
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

## CiA402 Homing Flow

The `home_crane` service runs the standard CiA402 homing sequence on top of the existing profile-position and profile-velocity modes.

### 1. Operation mode initialization: `0x6060` / `0x6061`

- The controller writes `0x6060:00 = 6` to request HM.
- It immediately reads `0x6061:00` and requires the feedback value to also be `6`.
- If `0x6061` does not report HM, the homing command is rejected before any motion trigger is sent.

### 2. Homing method selection: `0x6098`

- `Method 35`: in this project it is handled as "set current position as home" for absolute-encoder axes. The node writes `0x607C` (Home Offset), verifies that the actual position becomes approximately zero, and then stores the result through `0x1010`.
- `Method 32/33/34`: move the axis to find the encoder index (`Z`) signal and then latch the home position. These methods are appropriate when the machine must re-acquire a physical repeatable reference, but they require safe travel range, correct direction selection, and more careful commissioning.
- For absolute-encoder `Trolley` and `Hoist` axes, the safest default is `Method 35` because it avoids unnecessary travel and startup risk. Use `32/33/34` only when the mechanism still needs a physical reference pass to remove installation or transmission drift.

### 3. Homing motion parameters: `0x6099` / `0x609A`

- `0x6099:01` is the higher speed used while searching for the switch/index region.
- `0x6099:02` is the lower speed used for final zero capture and should be slower than `0x6099:01`.
- `0x609A:00` defines the homing acceleration.
- In this project these values come from `homing_fast_velocity`, `homing_slow_velocity`, and `homing_acceleration` in `config/crane_params.yaml`.

### 4. Start trigger: `0x6040`

- Before homing, the servo must already be in CiA402 `Operation Enabled` state (`0x6041 & 0x006F == 0x0027`).
- For `Method 32/33/34`, the actual homing trigger is the rising edge of controlword bit 4.
- The controller sends `0x000F -> 0x001F -> 0x000F` so bit 4 rises once and then clears again.
- `Method 35` does not wait for `0x6041 bit 12`; it directly updates `0x607C` and therefore avoids the "stuck at 0x0237" behavior seen on some drives.

### 5. Result monitoring: `0x6041`

- `Bit 12 = 1`: homing attained / completed.
- `Bit 13 = 1`: homing error.
- For `Method 32/33/34`, the controller polls `0x6041` until one of these conditions is met or a timeout occurs. Fault state (`bit 3`) is also treated as failure.
- For `Method 35`, success is determined by reading back `0x6064` after writing `0x607C` and confirming the position is close to zero.

### 6. Persistent storage: `0x1010`

- After successful homing, the controller can write `0x1010:01 = 0x65766173` (`"save"`) to store the resulting offset into non-volatile memory.
- This behavior is controlled by `homing_store_parameters`.
- After HM finishes, the node can restore the mode that was active before homing via `homing_restore_previous_mode`.

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
| /{namespace}/home_crane | std_srvs/srv/Trigger | Run CiA402 homing mode |
| /{namespace}/set_crane_mode | std_srvs/srv/SetBool | Set motor mode (true: position mode, false: velocity mode) |

## Parameter List

| Parameter Name | Type | Default Value | Description |
| -------------- | ---- | ------------- | ----------- |
| can_interface_name | string | "can0" | CAN interface name |
| node_id | string | "1" | CANopen node ID ("1", "2", "3", or "all") |
| gear_ratio | float | 1.0 | Gear ratio (automatically set: hoist=20.0, trolley=10.0, slewing=10.0) |
| auto_start | bool | true | Whether to auto-start motor |
| profile_velocity | int | 5 | Profile velocity (degrees/second) |
| profile_acceleration | int | 5 | Profile acceleration (degrees/second²) |
| profile_deceleration | int | 5 | Profile deceleration (degrees/second²) |
| homing_method | int | 35 | CiA402 homing method (`35` direct zero, `32/33/34` index-based) |
| homing_fast_velocity | float | 10.0 | `0x6099:01` fast search speed for HM |
| homing_slow_velocity | float | 2.0 | `0x6099:02` slow zero-capture speed for HM |
| homing_acceleration | float | 10.0 | `0x609A` homing acceleration |
| homing_timeout_s | double | 30.0 | HM completion timeout |
| homing_store_parameters | bool | true | Save homing offset via `0x1010` after success |
| homing_restore_previous_mode | bool | true | Restore the pre-homing mode after HM |

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
