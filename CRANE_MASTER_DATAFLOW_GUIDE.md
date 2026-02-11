# crane_master 完整数据流技术文档

> **版本**: 1.0  
> **日期**: 2026-02-01  
> **作者**: Technical Documentation Team  
> **项目**: Tower Crane CANopen Control System

---

## 📑 目录

1. [系统架构总览](#1-系统架构总览)
2. [数据流主线路](#2-数据流主线路)
3. [技术栈清单](#3-技术栈清单)
4. [Layer 1: 用户接口层](#4-layer-1-用户接口层)
5. [Layer 2: ROS 2 中间件层](#5-layer-2-ros-2-中间件层)
6. [Layer 3: 应用逻辑层](#6-layer-3-应用逻辑层)
7. [Layer 4: CANopen 协议层](#7-layer-4-canopen-协议层)
8. [Layer 5: 系统调用层](#8-layer-5-系统调用层)
9. [Layer 6: Linux 内核层](#9-layer-6-linux-内核层)
10. [Layer 7: 硬件物理层](#10-layer-7-硬件物理层)
11. [反向数据流（反馈路径）](#11-反向数据流反馈路径)
12. [完整数据流时序图](#12-完整数据流时序图)
13. [关键技术深度解析](#13-关键技术深度解析)
14. [性能分析与优化](#14-性能分析与优化)

---

## 1. 系统架构总览

### 1.1 系统分层架构

```
┌─────────────────────────────────────────────────────────────────────┐
│  Layer 7: 硬件物理层 (Hardware Physical Layer)                      │
│  - CAN Bus (ISO 11898)                                               │
│  - CAN Transceiver (TJA1050/MCP2551)                                 │
│  - Motor Drivers (CANopen CiA 402)                                   │
└────────────────────────────┬────────────────────────────────────────┘
                             │ CAN Frames (High/Low Voltage)
┌────────────────────────────▼────────────────────────────────────────┐
│  Layer 6: Linux 内核层 (Linux Kernel Layer)                          │
│  - SocketCAN Driver                                                   │
│  - Network Stack (can.ko, can-raw.ko)                               │
│  - Device Driver (can-dev.ko)                                        │
└────────────────────────────┬────────────────────────────────────────┘
                             │ Socket API (file descriptors)
┌────────────────────────────▼────────────────────────────────────────┐
│  Layer 5: 系统调用层 (System Call Layer)                             │
│  - socket(), bind(), write(), read()                                 │
│  - ioctl(), fcntl()                                                  │
│  - POSIX API                                                         │
└────────────────────────────┬────────────────────────────────────────┘
                             │ C++ API
┌────────────────────────────▼────────────────────────────────────────┐
│  Layer 4: CANopen 协议层 (CANopen Protocol Layer)                    │
│  - SDO (Service Data Object)                                         │
│  - PDO (Process Data Object)                                         │
│  - NMT (Network Management)                                          │
│  - SYNC (Synchronization)                                            │
│  - CiA 301 (Application Layer)                                       │
│  - CiA 402 (Device Profile for Drives)                              │
└────────────────────────────┬────────────────────────────────────────┘
                             │ C++ Methods
┌────────────────────────────▼────────────────────────────────────────┐
│  Layer 3: 应用逻辑层 (Application Logic Layer)                       │
│  - Motor Control State Machine                                       │
│  - Position/Velocity Control                                         │
│  - Error Handling                                                    │
│  - Unit Conversion                                                   │
└────────────────────────────┬────────────────────────────────────────┘
                             │ Callback Functions
┌────────────────────────────▼────────────────────────────────────────┐
│  Layer 2: ROS 2 中间件层 (ROS 2 Middleware Layer)                    │
│  - rclcpp (ROS Client Library)                                       │
│  - DDS (Data Distribution Service)                                   │
│  - Topics, Services, Actions                                         │
│  - Executors & Callbacks                                             │
└────────────────────────────┬────────────────────────────────────────┘
                             │ ROS 2 Messages
┌────────────────────────────▼────────────────────────────────────────┐
│  Layer 1: 用户接口层 (User Interface Layer)                          │
│  - Command Line (ros2 topic/service/action)                          │
│  - Joystick (STM32 + Serial)                                         │
│  - MoveIt (Motion Planning)                                          │
│  - Custom Applications                                                │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 数据流方向

#### 正向数据流 (命令路径)
```
User → ROS 2 → Application Logic → CANopen → SocketCAN → CAN Bus → Motor
```

#### 反向数据流 (反馈路径)
```
Motor → CAN Bus → SocketCAN → CANopen → Application Logic → ROS 2 → User
```

---

## 2. 数据流主线路

### 2.1 位置控制完整数据流

```
Step 1: User Input
  ├─ Command: ros2 topic pub /hoist/target_position std_msgs/Float32 "data: 90.0"
  └─ Data: 90.0 degrees

Step 2: ROS 2 DDS
  ├─ Serialize message to RTPS format
  ├─ Transport via UDP/Shared Memory
  └─ Deserialize in subscriber node

Step 3: Callback Trigger
  ├─ rclcpp Executor polls for messages
  ├─ Invoke position_callback()
  └─ Pass std_msgs::msg::Float32::SharedPtr

Step 4: Application Logic
  ├─ Extract angle: 90.0°
  ├─ Read current state via SDO
  │   ├─ Status Word (0x6041)
  │   └─ Operation Mode (0x6061)
  └─ Call go_to_position(90.0)

Step 5: Unit Conversion
  ├─ angle_to_position(90.0)
  ├─ Formula: (90/360) × (10000/10) = 250 units
  └─ Result: 250 command units

Step 6: SDO Configuration
  ├─ write_sdo(0x607A, 0x00, 250, 4)
  ├─ Build CAN Frame:
  │   ├─ ID: 0x601 (0x600 + node_id)
  │   ├─ DLC: 8
  │   └─ Data: [23 7A 60 00 FA 00 00 00]
  └─ write(can_socket_, frame, sizeof(frame))

Step 7: SocketCAN Processing
  ├─ Kernel receives write() system call
  ├─ Validate CAN frame
  ├─ Queue frame in TX buffer
  └─ Hardware transmits on CAN bus

Step 8: CAN Bus Physical
  ├─ Dominant/Recessive bits (0/1)
  ├─ Arbitration (ID 0x601)
  ├─ Bit stuffing
  └─ CRC check

Step 9: Motor Driver Reception
  ├─ CAN controller receives frame
  ├─ Filter by Node ID
  ├─ Parse SDO: Index 0x607A = Target Position
  └─ Store value: 250 units

Step 10: PDO Handshake
  ├─ Phase 1: Send 0x0F + position via RPDO1 → SYNC
  ├─ Phase 2: Send 0x1F + position via RPDO1 → SYNC (trigger)
  ├─ Phase 3: Send 0x0F + position via RPDO1 → SYNC (complete)
  └─ Motor starts movement

Step 11: Motor Execution
  ├─ Driver parses control word bit 4 (New Set Point)
  ├─ Calculate trajectory (velocity, acceleration)
  ├─ PWM control to motor phases
  └─ Move to target position

Step 12: Feedback (TPDO1)
  ├─ Motor sends TPDO1 every SYNC period
  ├─ CAN Frame: [0x181] [Status_Word] [Position]
  ├─ SocketCAN receives frame
  └─ Application reads via read()

Step 13: State Update
  ├─ Parse TPDO1 data
  ├─ Extract status_word (bit 10 = target reached)
  ├─ Extract actual_position (32-bit)
  └─ Update internal state

Step 14: ROS 2 Publishing
  ├─ Convert position to angle
  ├─ Publish to /hoist/crane_position
  └─ User receives feedback
```

### 2.2 速度控制数据流

```
User → target_velocity topic → velocity_callback()
  → set_velocity_pdo()
  → Check/Switch to Velocity Mode (MODE_PROFILE_VELOCITY = 3)
  → Convert velocity (deg/s → units/s)
  → Send via RPDO2 (0x300 + node_id)
  → [Control_Word][Target_Velocity_4_bytes]
  → Motor executes velocity control
  → Feedback via TPDO2
```

---

## 3. 技术栈清单

### 3.1 编程语言

| 语言 | 用途 | 文件类型 |
|------|------|---------|
| **C++14** | 核心控制逻辑 | `.cpp`, `.hpp` |
| **Python 3** | 工具脚本、摇杆接口 | `.py` |
| **CMake** | 构建系统 | `CMakeLists.txt` |
| **YAML** | 配置文件 | `.yml` |
| **XML** | 包描述 | `package.xml` |

### 3.2 ROS 2 组件

| 组件 | 版本 | 功能 |
|------|------|------|
| **rclcpp** | Humble | ROS 2 C++ 客户端库 |
| **std_msgs** | - | 标准消息类型 (Float32, String) |
| **std_srvs** | - | 标准服务类型 (Trigger, SetBool) |
| **geometry_msgs** | - | 几何消息 (Twist) |
| **control_msgs** | - | 控制消息 (FollowJointTrajectory) |
| **sensor_msgs** | - | 传感器消息 (JointState) |
| **rclcpp_action** | - | Action 服务器/客户端 |

### 3.3 Linux 系统组件

| 组件 | 功能 |
|------|------|
| **SocketCAN** | Linux 内核 CAN 总线支持 |
| **can-utils** | CAN 总线工具集 (candump, cansend) |
| **netlink** | 内核与用户空间通信 |
| **POSIX Threads** | 多线程支持 |
| **POSIX Sockets** | 网络套接字 API |

### 3.4 CANopen 标准

| 标准 | 编号 | 内容 |
|------|------|------|
| **CANopen Application Layer** | CiA 301 | 基础协议 (SDO, PDO, NMT, SYNC) |
| **Device Profile for Drives** | CiA 402 | 电机控制标准 (状态机, 对象字典) |
| **CAN Physical Layer** | ISO 11898 | CAN 总线物理层规范 |

### 3.5 硬件接口

| 硬件 | 接口 | 协议 |
|------|------|------|
| **CAN Adapter** | USB/PCIe | SocketCAN |
| **Motor Drivers** | CAN Bus | CANopen CiA 402 |
| **Joystick (STM32)** | Serial (USB) | UART (115200 baud) |

---

## 4. Layer 1: 用户接口层

### 4.1 技术：命令行接口 (CLI)

#### 工具：ros2 CLI
```bash
# 位置控制
ros2 topic pub /hoist/target_position std_msgs/msg/Float32 "data: 90.0" --once

# 速度控制
ros2 topic pub /slewing/target_velocity std_msgs/msg/Float32 "data: 10.0" --once

# 启动电机
ros2 service call /hoist/start_crane std_srvs/srv/Trigger

# 模式切换
ros2 service call /hoist/set_crane_mode std_srvs/srv/SetBool "data: true"
```

#### 技术细节
- **工具**: `ros2` Python 脚本
- **传输**: Unix Domain Socket / UDP (取决于 DDS 配置)
- **序列化**: CDR (Common Data Representation)
- **发现**: DDS Simple Discovery Protocol

### 4.2 技术：摇杆接口

#### 硬件：STM32 + UART

**数据格式**:
```
Serial Frame: "x,y,sw\n"
Example: "512,768,0\n"
```

**Python 脚本**: `joystick_bridge.py`
```python
import serial
import rclpy
from std_msgs.msg import Float32

# 打开串口
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

# 读取数据
line = ser.readline().decode('utf-8').strip()
x, y, sw = map(int, line.split(','))

# 发布到 ROS 2
pub_slewing.publish(Float32(data=angle_from_joystick(x)))
```

**技术栈**:
- **pyserial**: Python 串口库
- **rclpy**: ROS 2 Python 客户端库
- **UART 协议**: 8N1 (8 data bits, No parity, 1 stop bit)

### 4.3 技术：Action 接口 (同步轨迹)

#### Action 定义
```
control_msgs/action/FollowJointTrajectory

Goal:
  trajectory:
    joint_names: [slewing_joint, trolley_joint, hook_joint]
    points:
      - positions: [10.0, 1.0, 20.0]
        time_from_start: {sec: 2, nanosec: 0}

Feedback:
  desired: trajectory_msgs/JointTrajectoryPoint
  actual: trajectory_msgs/JointTrajectoryPoint
  error: trajectory_msgs/JointTrajectoryPoint

Result:
  error_code: int32
  error_string: string
```

---

## 5. Layer 2: ROS 2 中间件层

### 5.1 技术：DDS (Data Distribution Service)

#### DDS 实现
- **默认**: Fast-DDS (eProsima)
- **可选**: Cyclone DDS, RTI Connext

#### 数据发现流程
```
1. Publisher announces topic
   ├─ Topic Name: "/hoist/target_position"
   ├─ Type: "std_msgs/msg/Float32"
   └─ QoS: Reliability, Durability, History

2. Subscriber discovers publisher
   ├─ Match QoS policies
   ├─ Establish connection
   └─ Subscribe to data stream

3. Data transmission
   ├─ Serialize message (CDR)
   ├─ Fragment if needed (UDP MTU ~1500 bytes)
   ├─ Transport via UDP multicast/unicast
   └─ Deserialize in subscriber
```

### 5.2 技术：rclcpp 执行器

#### 单线程执行器
```cpp
rclcpp::init(argc, argv);
auto node = std::make_shared<CANopenROS2>();

// 创建订阅器
position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "target_position",
    10,  // QoS 队列深度
    std::bind(&CANopenROS2::position_callback, this, std::placeholders::_1)
);

// 自旋执行器（阻塞）
rclcpp::spin(node);
```

#### 执行器工作流程
```
while(rclcpp::ok()) {
    1. Wait for work (epoll on DDS sockets)
    2. Receive message from DDS
    3. Lookup callback in subscription map
    4. Invoke position_callback(msg)
    5. Return to wait state
}
```

### 5.3 技术：定时器

#### 创建定时器
```cpp
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&CANopenROS2::receive_can_frames, this)
);
```

#### 定时器实现
- **底层**: `timerfd_create()` (Linux)
- **精度**: 毫秒级（非实时）
- **触发**: epoll 检测 timerfd 可读
- **回调**: 在主线程中执行

---

## 6. Layer 3: 应用逻辑层

### 6.1 技术：状态机管理

#### CiA 402 状态机
```
State 0: Not Ready to Switch On
    ↓ (Power on)
State 1: Switch On Disabled
    ↓ (Shutdown: 0x06)
State 2: Ready to Switch On
    ↓ (Switch On: 0x07)
State 3: Switched On
    ↓ (Enable Operation: 0x0F)
State 4: Operation Enabled ← 正常工作状态
    ↓ (Disable Operation: 0x07)
State 3: Switched On
    ↓ (Quick Stop / Fault)
State 5: Quick Stop Active / State 6: Fault
```

#### 状态转换代码
```cpp
void CANopenROS2::enable_motor() {
    // 1. Shutdown (进入 State 2)
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 2. Switch On (进入 State 3)
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 3. Enable Operation (进入 State 4)
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // 验证状态
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    if ((status_word & 0x006F) == 0x0027) {
        RCLCPP_INFO("电机已成功使能 (Operation enabled)");
    }
}
```

### 6.2 技术：单位转换

#### 位置转换
```cpp
// 角度 → 命令单位
int32_t CANopenROS2::angle_to_position(float angle) {
    // Formula: (angle / 360°) × (target_units_per_rev / gear_ratio)
    // Example: (90 / 360) × (10000 / 10) = 250 units
    int32_t position = static_cast<int32_t>(angle * units_per_degree_);
    return position;
}

// 命令单位 → 角度
float CANopenROS2::position_to_angle(int32_t position) {
    // Formula: position × (gear_ratio / target_units_per_rev) × 360°
    float angle = static_cast<float>(position) * degrees_per_unit_;
    return angle;
}
```

#### 预计算比例
```cpp
// 构造函数中计算
units_per_degree_ = (target_units_per_rev_ / gear_ratio_) / 360.0f;
degrees_per_unit_ = (gear_ratio_ / target_units_per_rev_) * 360.0f;

// 示例：gear_ratio = 10.0, target_units_per_rev = 10000
units_per_degree_ = (10000 / 10.0) / 360.0 = 2.777...
degrees_per_unit_ = (10.0 / 10000) * 360.0 = 0.36
```

### 6.3 技术：错误处理

#### 故障检测
```cpp
void CANopenROS2::check_and_clear_error() {
    // 读取错误寄存器 (0x1001)
    int32_t error_register = read_sdo(0x1001, 0x00);
    if (error_register > 0) {
        RCLCPP_ERROR("Error Register (0x1001): 0x%02X", error_register);
        
        // 读取制造商错误码 (0x603F)
        int32_t manufacturer_error = read_sdo(0x603F, 0x00);
        if (manufacturer_error > 0) {
            RCLCPP_ERROR("Manufacturer Error (0x603F): 0x%04X", manufacturer_error);
        }
    }
}
```

#### 故障恢复
```cpp
void CANopenROS2::clear_fault() {
    // 1. 发送故障复位命令 (0x80)
    set_control_word(CONTROL_FAULT_RESET);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. 关闭 (0x06)
    set_control_word(CONTROL_SHUTDOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 3. 重新使能
    enable_motor();
}
```

---

## 7. Layer 4: CANopen 协议层

### 7.1 技术：对象字典 (Object Dictionary)

#### 对象字典索引
```cpp
// CiA 402 标准对象
#define OD_CONTROL_WORD          0x6040  // 控制字
#define OD_STATUS_WORD           0x6041  // 状态字
#define OD_OPERATION_MODE        0x6060  // 操作模式设定
#define OD_OPERATION_MODE_DISPLAY 0x6061 // 操作模式显示
#define OD_TARGET_POSITION       0x607A  // 目标位置
#define OD_TARGET_VELOCITY       0x60FF  // 目标速度
#define OD_ACTUAL_POSITION       0x6064  // 实际位置
#define OD_ACTUAL_VELOCITY       0x606C  // 实际速度
#define OD_PROFILE_VELOCITY      0x6081  // 轮廓速度
#define OD_PROFILE_ACCELERATION  0x6083  // 轮廓加速度
#define OD_PROFILE_DECELERATION  0x6084  // 轮廓减速度

// CiA 301 通用对象
#define OD_CYCLE_PERIOD          0x1006  // 通信周期
#define OD_GEAR_RATIO            0x6091  // 齿轮比
#define OD_ERROR_REGISTER        0x1001  // 错误寄存器
```

### 7.2 技术：SDO (Service Data Object)

#### SDO 帧格式

**下载请求 (Write)**:
```
CAN ID: 0x600 + Node ID
DLC: 8 bytes

Byte 0: Command Specifier
  - 0x2F: 1-byte download
  - 0x2B: 2-byte download
  - 0x27: 3-byte download
  - 0x23: 4-byte download

Byte 1: Index Low Byte
Byte 2: Index High Byte
Byte 3: Sub-index
Byte 4-7: Data (little-endian)
```

**上传请求 (Read)**:
```
CAN ID: 0x600 + Node ID
DLC: 8 bytes

Byte 0: 0x40 (Upload Request)
Byte 1: Index Low Byte
Byte 2: Index High Byte
Byte 3: Sub-index
Byte 4-7: Reserved (0x00)
```

**SDO 响应**:
```
CAN ID: 0x580 + Node ID
DLC: 8 bytes

Byte 0: Command Response
  - 0x60: Upload success (1-byte)
  - 0x5B: Upload success (2-byte)
  - 0x57: Upload success (3-byte)
  - 0x53: Upload success (4-byte)
  - 0x80: Abort (error)

Byte 1-3: Index + Sub-index (echo)
Byte 4-7: Data or Abort Code
```

#### SDO 写入实现
```cpp
void CANopenROS2::write_sdo(uint16_t index, uint8_t subindex, 
                             int32_t data, uint8_t size) {
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;  // 0x600 + node_id
    frame.can_dlc = 8;
    
    // 构造命令字节
    uint8_t command = 0x22;  // 4-byte download
    if (size == 1) command |= 0x0F;
    else if (size == 2) command |= 0x0B;
    else if (size == 4) command |= 0x03;
    
    frame.data[0] = command;
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    frame.data[4] = data & 0xFF;
    frame.data[5] = (data >> 8) & 0xFF;
    frame.data[6] = (data >> 16) & 0xFF;
    frame.data[7] = (data >> 24) & 0xFF;
    
    // 发送到 SocketCAN
    write(can_socket_, &frame, sizeof(struct can_frame));
}
```

#### SDO 读取实现（带同步机制）
```cpp
int32_t CANopenROS2::read_sdo(uint16_t index, uint8_t subindex) {
    // 1. 设置期望响应
    {
        std::lock_guard<std::mutex> lock(sdo_mutex_);
        sdo_response_received_ = false;
        expected_sdo_index_ = index;
        expected_sdo_subindex_ = subindex;
        sdo_read_value_ = 0;
    }
    
    // 2. 发送上传请求
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;
    frame.can_dlc = 8;
    frame.data[0] = 0x40;  // Upload request
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    write(can_socket_, &frame, sizeof(struct can_frame));
    
    // 3. 等待响应（轮询 + 超时）
    int retry = 0;
    const int max_retries = 50;  // 500ms timeout
    
    while (retry < max_retries) {
        for (int i = 0; i < 20; i++) {
            receive_can_frames();  // 尝试接收
            
            {
                std::lock_guard<std::mutex> lock(sdo_mutex_);
                if (sdo_response_received_) {
                    return sdo_read_value_;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        retry++;
    }
    
    // 超时
    RCLCPP_WARN("SDO read timeout");
    return 0;
}
```

### 7.3 技术：PDO (Process Data Object)

#### PDO 映射配置

**TxPDO1 (反馈数据)**:
```
COB-ID: 0x180 + Node ID
Transmission Type: 0x01 (Synchronous)
Mapping:
  - Byte 0-1: Status Word (0x6041)
  - Byte 2-5: Actual Position (0x6064)
```

**RxPDO1 (位置控制)**:
```
COB-ID: 0x200 + Node ID
Transmission Type: 0xFF (Asynchronous)
Mapping:
  - Byte 0-1: Control Word (0x6040)
  - Byte 2-5: Target Position (0x607A)
```

**RxPDO2 (速度控制)**:
```
COB-ID: 0x300 + Node ID
Transmission Type: 0xFF (Asynchronous)
Mapping:
  - Byte 0-1: Control Word (0x6040)
  - Byte 2-5: Target Velocity (0x60FF)
```

#### PDO 配置代码
```cpp
void CANopenROS2::configure_pdo() {
    // 进入 Pre-Operational 状态
    send_nmt_command(NMT_STOP_REMOTE_NODE);
    
    // === 配置 TxPDO1 ===
    uint32_t txpdo1_cob_id = COB_TPDO1 + node_id_;
    
    // 1. 禁用 TxPDO1
    write_sdo(0x1800, 0x01, txpdo1_cob_id | 0x80000000, 4);
    
    // 2. 清除映射
    write_sdo(0x1A00, 0x00, 0x00, 1);
    
    // 3. 映射 Status Word (16 bits)
    write_sdo(0x1A00, 0x01, 0x60410010, 4);
    
    // 4. 映射 Actual Position (32 bits)
    write_sdo(0x1A00, 0x02, 0x60640020, 4);
    
    // 5. 设置对象数量 = 2
    write_sdo(0x1A00, 0x00, 0x02, 1);
    
    // 6. 设置传输类型 (0x01 = Synchronous)
    write_sdo(0x1800, 0x02, 0x01, 1);
    
    // 7. 启用 TxPDO1
    write_sdo(0x1800, 0x01, txpdo1_cob_id, 4);
    
    // === 配置 RxPDO1 (类似流程) ===
    // ...
    
    // 返回 Operational 状态
    send_nmt_command(NMT_START_REMOTE_NODE);
}
```

#### PDO 位置控制握手
```cpp
void CANopenROS2::go_to_position(float angle) {
    int32_t position = angle_to_position(angle);
    
    // Phase 1: Enable Operation + Position
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 6;
    frame.data[0] = 0x0F;  // Enable Operation
    frame.data[1] = 0x00;
    frame.data[2] = position & 0xFF;
    frame.data[3] = (position >> 8) & 0xFF;
    frame.data[4] = (position >> 16) & 0xFF;
    frame.data[5] = (position >> 24) & 0xFF;
    
    write(can_socket_, &frame, sizeof(frame));
    send_sync_frame();  // COB-ID 0x080
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // Phase 2: New Set Point (上升沿触发)
    frame.data[0] = 0x1F;  // Enable Op + New Set Point (bit 4)
    write(can_socket_, &frame, sizeof(frame));
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // Phase 3: 恢复 (完成握手)
    frame.data[0] = 0x0F;
    write(can_socket_, &frame, sizeof(frame));
    send_sync_frame();
}
```

### 7.4 技术：NMT (Network Management)

#### NMT 状态转换
```
Initialization → Bootup → Pre-Operational → Operational
                    ↑           ↓
                    └─── Reset ──┘
```

#### NMT 命令
```cpp
#define NMT_START_REMOTE_NODE    0x01  // Pre-Op → Operational
#define NMT_STOP_REMOTE_NODE     0x02  // Operational → Pre-Op
#define NMT_RESET_NODE           0x81  // 复位节点
#define NMT_RESET_COMM           0x82  // 复位通信
```

#### NMT 发送
```cpp
void CANopenROS2::send_nmt_command(uint8_t command) {
    struct can_frame frame;
    frame.can_id = COB_NMT;  // 0x000
    frame.can_dlc = 2;
    frame.data[0] = command;  // NMT command
    frame.data[1] = node_id_; // Target node ID
    
    write(can_socket_, &frame, sizeof(frame));
}
```

### 7.5 技术：SYNC (同步对象)

#### SYNC 帧格式
```
CAN ID: 0x080
DLC: 0 (无数据)
Data: (empty)
```

#### SYNC 作用
- **同步 PDO 传输**：所有节点在同一时刻执行
- **时间戳**：可选携带时间戳（extended SYNC）
- **周期**：由对象 0x1006 定义（单位：微秒）

#### SYNC 发送
```cpp
void CANopenROS2::send_sync_frame() {
    struct can_frame frame;
    frame.can_id = COB_SYNC;  // 0x080
    frame.can_dlc = 0;        // 无数据
    
    write(can_socket_, &frame, sizeof(frame));
}
```

---

## 8. Layer 5: 系统调用层

### 8.1 技术：Socket API

#### SocketCAN 初始化
```cpp
void CANopenROS2::init_can_socket() {
    // 1. 创建套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        perror("socket");
        return;
    }
    
    // 2. 获取接口索引
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());  // "can0"
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        return;
    }
    
    // 3. 绑定到 CAN 接口
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return;
    }
    
    // 4. 设置非阻塞模式
    int flags = fcntl(can_socket_, F_GETFL, 0);
    flags |= O_NONBLOCK;
    fcntl(can_socket_, F_SETFL, flags);
}
```

#### 系统调用详解

**socket()**:
```c
#include <sys/socket.h>

int socket(int domain, int type, int protocol);

domain:   PF_CAN (Protocol Family CAN)
type:     SOCK_RAW (Raw socket, 直接访问 CAN 帧)
protocol: CAN_RAW (RAW CAN protocol)

返回: 文件描述符 (>=0) 或 -1 (失败)
```

**ioctl()**:
```c
#include <sys/ioctl.h>

int ioctl(int fd, unsigned long request, ...);

fd:      socket 文件描述符
request: SIOCGIFINDEX (获取接口索引)
arg:     struct ifreq* (包含接口名称)

功能: 将接口名 "can0" 转换为内核索引号
```

**bind()**:
```c
#include <sys/socket.h>

int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen);

sockfd:  socket 文件描述符
addr:    sockaddr_can* (包含 CAN 地址信息)
addrlen: sizeof(struct sockaddr_can)

功能: 将 socket 绑定到特定的 CAN 接口
```

**fcntl()**:
```c
#include <fcntl.h>

int fcntl(int fd, int cmd, ...);

fd:  文件描述符
cmd: F_GETFL (获取标志) / F_SETFL (设置标志)
arg: 标志位 (O_NONBLOCK = 非阻塞)

功能: 设置 socket 为非阻塞模式，避免 read() 阻塞
```

### 8.2 技术：CAN 帧读写

#### CAN 帧结构
```c
#include <linux/can.h>

struct can_frame {
    canid_t can_id;   // 32-bit CAN ID + EFF/RTR/ERR flags
    __u8    can_dlc;  // Data Length Code (0-8)
    __u8    __pad;    // Padding
    __u8    __res0;   // Reserved
    __u8    __res1;   // Reserved
    __u8    data[8];  // Data bytes
};

// CAN ID 组成 (Standard Frame, 11-bit)
// Bit 31:   ERR (Error Frame)
// Bit 30:   RTR (Remote Transmission Request)
// Bit 29:   EFF (Extended Frame Format)
// Bit 28-11: Reserved
// Bit 10-0:  CAN Identifier (0x000 - 0x7FF)
```

#### 写入 CAN 帧
```cpp
ssize_t write(int fd, const void *buf, size_t count);

// 示例
struct can_frame frame;
frame.can_id = 0x601;  // SDO Request to Node 1
frame.can_dlc = 8;
frame.data[0] = 0x23;  // 4-byte download
// ... 填充其他数据

ssize_t nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
if (nbytes != sizeof(struct can_frame)) {
    perror("write");
}
```

#### 读取 CAN 帧
```cpp
ssize_t read(int fd, void *buf, size_t count);

// 示例
struct can_frame frame;
ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // 非阻塞模式，无数据可读
        return;
    }
    perror("read");
} else if (nbytes == sizeof(struct can_frame)) {
    // 成功读取完整帧
    printf("Received CAN ID: 0x%03X\n", frame.can_id);
    printf("DLC: %d\n", frame.can_dlc);
    printf("Data: ");
    for (int i = 0; i < frame.can_dlc; i++) {
        printf("%02X ", frame.data[i]);
    }
    printf("\n");
}
```

---

## 9. Layer 6: Linux 内核层

### 9.1 技术：SocketCAN 驱动架构

#### SocketCAN 内核模块
```
/lib/modules/$(uname -r)/kernel/net/can/
├── can.ko              # CAN 协议核心
├── can-raw.ko          # RAW socket 支持
├── can-bcm.ko          # Broadcast Manager
├── can-gw.ko           # CAN Gateway
└── can-isotp.ko        # ISO-TP 协议
```

#### 加载模块
```bash
# 加载 CAN 核心模块
sudo modprobe can

# 加载 RAW socket 支持
sudo modprobe can_raw

# 查看已加载模块
lsmod | grep can
```

#### CAN 设备驱动
```
/lib/modules/$(uname -r)/kernel/drivers/net/can/
├── dev/
│   └── can-dev.ko      # CAN 设备核心
├── usb/
│   ├── peak_usb/       # PEAK USB-CAN
│   └── kvaser_usb/     # Kvaser USB-CAN
└── spi/
    └── mcp251x.ko      # MCP2515 SPI CAN
```

### 9.2 技术：CAN 接口配置

#### 虚拟 CAN (vcan)
```bash
# 加载 vcan 驱动
sudo modprobe vcan

# 创建虚拟 CAN 接口
sudo ip link add dev vcan0 type vcan

# 启动接口
sudo ip link set up vcan0

# 查看接口状态
ip -details link show vcan0
```

#### 真实 CAN (can0)
```bash
# 设置波特率 (1 Mbps)
sudo ip link set can0 type can bitrate 1000000

# 启动接口
sudo ip link set up can0

# 查看统计信息
ip -s link show can0
```

#### 高级配置
```bash
# 设置采样点 (sample point)
sudo ip link set can0 type can bitrate 500000 sample-point 0.875

# 设置三次采样
sudo ip link set can0 type can bitrate 500000 triple-sampling on

# 设置重启时间 (自动从 bus-off 恢复)
sudo ip link set can0 type can restart-ms 100

# 查看详细配置
ip -details link show can0
```

### 9.3 技术：CAN 帧处理流程

#### 发送路径 (TX)
```
Application (write)
    ↓
Kernel: Socket Layer (net/socket.c)
    ↓
Kernel: CAN Protocol (net/can/raw.c)
    - can_raw_sendmsg()
    ↓
Kernel: CAN Core (net/can/af_can.c)
    - can_send()
    ↓
Kernel: Network Device Layer (net/core/dev.c)
    - dev_queue_xmit()
    ↓
Kernel: CAN Device Driver (drivers/net/can/)
    - can_put_echo_skb()
    - Hardware TX register
    ↓
Hardware: CAN Controller
    ↓
CAN Bus (Physical)
```

#### 接收路径 (RX)
```
CAN Bus (Physical)
    ↓
Hardware: CAN Controller
    - Interrupt (IRQ)
    ↓
Kernel: CAN Device Driver
    - can_rx_offload()
    ↓
Kernel: Network Device Layer
    - netif_receive_skb()
    ↓
Kernel: CAN Core
    - can_rcv()
    - can_rcv_filter()
    ↓
Kernel: CAN Protocol (RAW)
    - can_raw_rcv()
    - skb_queue_tail()
    ↓
Kernel: Socket Layer
    - sock_read_ready()
    ↓
Application (read)
```

### 9.4 技术：CAN 错误处理

#### 错误类型
```c
#define CAN_ERR_TX_TIMEOUT    0x00000001U  // TX timeout
#define CAN_ERR_LOSTARB       0x00000002U  // Lost arbitration
#define CAN_ERR_CRTL          0x00000004U  // Controller problems
#define CAN_ERR_PROT          0x00000008U  // Protocol violations
#define CAN_ERR_TRX           0x00000010U  // Transceiver problems
#define CAN_ERR_ACK           0x00000020U  // No ACK on transmission
#define CAN_ERR_BUSOFF        0x00000040U  // Bus off
#define CAN_ERR_BUSERROR      0x00000080U  // Bus error
#define CAN_ERR_RESTARTED     0x00000100U  // Controller restarted
```

#### 错误帧接收
```cpp
struct can_frame frame;
read(can_socket_, &frame, sizeof(frame));

if (frame.can_id & CAN_ERR_FLAG) {
    // 这是错误帧
    if (frame.can_id & CAN_ERR_BUSOFF) {
        RCLCPP_ERROR("CAN bus off!");
    }
    if (frame.can_id & CAN_ERR_ACK) {
        RCLCPP_ERROR("No ACK received!");
    }
}
```

---

## 10. Layer 7: 硬件物理层

### 10.1 技术：CAN 总线物理层 (ISO 11898)

#### 差分信号
```
CAN_H (High): 3.5V (Dominant), 2.5V (Recessive)
CAN_L (Low):  1.5V (Dominant), 2.5V (Recessive)

Differential Voltage:
  Dominant:  CAN_H - CAN_L = 2.0V (逻辑 0)
  Recessive: CAN_H - CAN_L = 0.0V (逻辑 1)
```

#### 拓扑结构
```
┌─────────┐     ┌─────────┐     ┌─────────┐
│ Node 1  │     │ Node 2  │     │ Node 3  │
│ (Hook)  │     │(Trolley)│     │(Slewing)│
└────┬────┘     └────┬────┘     └────┬────┘
     │               │               │
     │    CAN_H      │               │
     ├───────────────┼───────────────┤
     │               │               │
     │    CAN_L      │               │
     ├───────────────┼───────────────┤
     │               │               │
    120Ω            (stub)          120Ω
Termination                     Termination
```

#### 终端电阻
- **位置**: 总线两端
- **阻值**: 120Ω
- **作用**: 匹配阻抗，防止信号反射
- **连接**: CAN_H 和 CAN_L 之间

### 10.2 技术：CAN 帧格式

#### 标准帧 (Standard Frame, 11-bit ID)
```
┌───────┬─────┬───┬──────┬────┬──────┬─────┬───────┬────┬────┬────┐
│  SOF  │ ID  │RTR│ IDE  │ r0 │ DLC  │DATA │  CRC  │ACK │EOF │IFS │
│ 1 bit │11bit│1b │ 1bit │1bit│4bits │0-8B │15bits│2bit│7bit│3bit│
└───────┴─────┴───┴──────┴────┴──────┴─────┴───────┴────┴────┴────┘

SOF: Start of Frame (Dominant)
ID:  Identifier (11 bits)
RTR: Remote Transmission Request
IDE: Identifier Extension (0 = Standard Frame)
r0:  Reserved
DLC: Data Length Code (0-8)
DATA: Data Field (0-8 bytes)
CRC: Cyclic Redundancy Check
ACK: Acknowledge
EOF: End of Frame (7 Recessive bits)
IFS: Inter-Frame Space (3 Recessive bits)
```

#### 位时序
```
Nominal Bit Time = 1 / Bit Rate

Example: 1 Mbps
Bit Time = 1 / 1,000,000 = 1 μs

Bit Segments:
┌──────┬─────┬──────┬──────┐
│ SYNC │ PS1 │ PS2  │ SJW  │
└──────┴─────┴──────┴──────┘

SYNC: Synchronization Segment (1 TQ)
PS1:  Phase Segment 1 (1-8 TQ)
PS2:  Phase Segment 2 (1-8 TQ)
SJW:  Synchronization Jump Width (1-4 TQ)

TQ: Time Quantum (最小时间单位)
```

### 10.3 技术：CAN 收发器 (Transceiver)

#### 常用芯片
- **TJA1050**: NXP, 1 Mbps
- **MCP2551**: Microchip, 1 Mbps
- **SN65HVD230**: TI, 3.3V, 1 Mbps

#### 电路连接
```
       MCU                    Transceiver                CAN Bus
┌───────────┐              ┌──────────┐
│           │              │          │
│    TX  ───┼──────────────┤ TXD      │
│           │              │          │
│    RX  ───┼──────────────┤ RXD      │
│           │              │          │
│   GND  ───┼──────────────┤ GND      │
│           │              │          │
│   3.3V ───┼──────────────┤ VCC      │
│           │              │          ├───── CAN_H
└───────────┘              │   CANH   │
                           │          ├───── CAN_L
                           │   CANL   │
                           │          │
                           └──────────┘
```

### 10.4 技术：仲裁机制 (Arbitration)

#### 非破坏性仲裁
```
当多个节点同时发送时，ID 越小优先级越高

Example:
Node 1 发送 ID 0x181 (0001 1000 0001)
Node 2 发送 ID 0x201 (0010 0000 0001)
Node 3 发送 ID 0x601 (0110 0000 0001)

比特 0:  所有节点发送 0 (Dominant)
比特 1:  所有节点发送 0
比特 2:  所有节点发送 0
比特 3:  Node 1=1, Node 2=1, Node 3=1 (Recessive)
比特 4:  Node 1=1, Node 2=0, Node 3=1
         → Node 2 发送 0 (Dominant)，胜出
         → Node 1 和 Node 3 检测到冲突，停止发送

结果: Node 2 (ID 0x201) 获得总线
```

---

## 11. 反向数据流（反馈路径）

### 11.1 电机反馈生成

#### 电机驱动器内部
```
1. 编码器读取
   ├─ 增量编码器 (A/B/Z 信号)
   ├─ 绝对编码器 (SSI/EnDat)
   └─ 位置计算: pulses → command units

2. 状态更新
   ├─ Status Word (0x6041)
   │   ├─ Bit 10: Target Reached
   │   ├─ Bit 3: Fault
   │   └─ Bit 2: Operation Enabled
   ├─ Actual Position (0x6064)
   └─ Actual Velocity (0x606C)

3. PDO 映射
   ├─ 将对象字典值映射到 TPDO1
   └─ 等待 SYNC 信号

4. TPDO 发送
   ├─ 接收 SYNC (0x080)
   ├─ 触发 TPDO1 传输
   └─ CAN ID: 0x180 + Node ID
```

### 11.2 SocketCAN 接收

#### 中断处理
```
1. CAN Controller 接收帧
   ├─ 硬件过滤 (ID filter)
   ├─ 存储到 RX FIFO
   └─ 触发中断 (IRQ)

2. 驱动 ISR (Interrupt Service Routine)
   ├─ 读取硬件寄存器
   ├─ 构造 struct sk_buff
   ├─ 调用 netif_receive_skb()
   └─ 返回 (IRQ 完成)

3. 软中断 (SoftIRQ)
   ├─ 处理网络包
   ├─ 调用 CAN 协议层
   └─ 唤醒等待的 socket

4. 应用层唤醒
   ├─ epoll/select 检测到可读
   └─ read() 返回数据
```

### 11.3 应用层处理

#### 接收循环
```cpp
void CANopenROS2::receive_can_frames() {
    struct can_frame frame;
    ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
    
    if (nbytes < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR("Read error: %s", strerror(errno));
        }
        return;
    }
    
    // 1. 提取 COB-ID 和 Node ID
    uint32_t cob_id = frame.can_id & 0x780;
    uint8_t node_id = frame.can_id & 0x7F;
    
    if (node_id != node_id_) {
        return;  // 不是我们的节点
    }
    
    // 2. 根据 COB-ID 分类处理
    if (frame.can_id == (COB_TSDO + node_id_)) {
        // SDO 响应
        handle_sdo_response(frame);
    } else if (cob_id == COB_TPDO1) {
        // TPDO1 反馈
        handle_tpdo1(frame);
    }
}
```

#### TPDO1 处理
```cpp
void handle_tpdo1(const struct can_frame& frame) {
    if (frame.can_dlc < 6) return;
    
    // 解析状态字 (Byte 0-1)
    uint16_t status_word = frame.data[0] | (frame.data[1] << 8);
    status_word_ = status_word;
    
    // 解析实际位置 (Byte 2-5)
    int32_t position = frame.data[2] | 
                      (frame.data[3] << 8) |
                      (frame.data[4] << 16) |
                      (frame.data[5] << 24);
    position_ = position;
    
    // 转换为角度
    float angle = position_to_angle(position);
    
    // 发布到 ROS 2
    auto msg = std_msgs::msg::Float32();
    msg.data = angle;
    position_pub_->publish(msg);
    
    // 检查目标到达
    if (status_word & 0x0400) {
        RCLCPP_INFO("Target position reached!");
    }
}
```

### 11.4 ROS 2 发布

#### 发布器机制
```cpp
// 创建发布器
position_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "crane_position", 
    10  // QoS 队列深度
);

// 发布消息
auto msg = std_msgs::msg::Float32();
msg.data = angle;
position_pub_->publish(msg);
```

#### DDS 传输
```
1. Serialize message (CDR)
   ├─ Float32: 4 bytes
   └─ RTPS header: ~20 bytes

2. Transport (UDP/SHM)
   ├─ Local: Shared Memory (iceoryx)
   └─ Remote: UDP Multicast (239.255.0.x)

3. Subscriber receives
   ├─ Deserialize message
   ├─ Invoke callback (if subscribed)
   └─ Update topic cache
```

---

## 12. 完整数据流时序图

### 12.1 位置控制完整时序

```
Time    User        ROS2        App Logic   CANopen     SocketCAN   CAN Bus     Motor
────────────────────────────────────────────────────────────────────────────────────
t=0     pub topic
        ────────→   DDS
                    ────────→   callback
                                ────────→   read_sdo
                                            ────────→   write()
                                                        ────────→   [0x601]
                                                                    ────────→   SDO RX
                                                                                Process
                                                                    ←────────   [0x581]
                                                        ←────────   read()
                                            ←────────   response
                                ←────────   status

t=10ms                          angle_to
                                _position
                                ────────→   write_sdo
                                            ────────→   write()
                                                        ────────→   [0x601]
                                                                    ────────→   Store
                                                                                0x607A

t=20ms                          PDO Phase1
                                ────────→   RPDO1
                                            ────────→   write()
                                                        ────────→   [0x201]
                                                                    ────────→   Buffer

                                            send_sync
                                            ────────→   write()
                                                        ────────→   [0x080]
                                                                    ────────→   Wait

t=40ms                          PDO Phase2
                                ────────→   RPDO1(0x1F)
                                            ────────→   write()
                                                        ────────→   [0x201]
                                                                    ────────→   Trigger!

                                            send_sync
                                            ────────→   write()
                                                        ────────→   [0x080]
                                                                    ────────→   EXECUTE
                                                                                Start
                                                                                Moving

t=60ms                          PDO Phase3
                                ────────→   RPDO1(0x0F)
                                            ────────→   write()
                                                        ────────→   [0x201]
                                                                    ────────→   Continue

t=70ms  (Timer 10ms)            receive_
                                can_frames
                                ────────→   read()
                                            ────────→   read()
                                                        ←────────   [0x181]
                                                                    ←────────   TPDO1
                                            ←────────   TPDO
                                ←────────   Status+Pos
                                
                                publish
                                ────────→   ROS2 DDS
                    ←────────   topic

t=200ms                         receive_
                                can_frames
                                ────────→   read()
                                            ────────→   read()
                                                        ←────────   [0x181]
                                                                    ←────────   Moving...

t=2000ms                        receive_
                                can_frames
                                ────────→   read()
                                            ────────→   read()
                                                        ←────────   [0x181]
                                                                    ←────────   Reached!
                                                                                (Bit 10)
                                ←────────   Target
                                            Reached

        ←────────   echo topic
```

### 12.2 速度控制时序

```
Time    User        ROS2        App Logic   CANopen     SocketCAN   CAN Bus     Motor
────────────────────────────────────────────────────────────────────────────────────
t=0     pub velocity
        ────────→   DDS
                    ────────→   velocity_
                                callback
                                ────────→   check_mode
                                            read_sdo(0x6061)
                                            ────────→   write()
                                                        ────────→   [0x601]
                                                                    ────────→   Read Mode

                                ←────────   Mode=1
                                            (Position)

t=100ms                         switch_mode
                                ────────→   write_sdo
                                            (0x6060=3)
                                            ────────→   write()
                                                        ────────→   [0x601]
                                                                    ────────→   Set Mode=3

t=300ms                         velocity_
                                to_units
                                ────────→   send_RPDO2
                                            ────────→   write()
                                                        ────────→   [0x303]
                                                                    ────────→   Velocity
                                                                                Control

t=310ms                                                             ←────────   [0x181]
                                                                    ←────────   Feedback
                                ←────────   Position
                                            update
```

---

## 13. 关键技术深度解析

### 13.1 线程与并发

#### 主线程
```cpp
rclcpp::init(argc, argv);
auto node = std::make_shared<CANopenROS2>();
rclcpp::spin(node);  // 阻塞，处理回调
rclcpp::shutdown();
```

#### 定时器线程
```
Timer 1 (10ms):  receive_can_frames()
Timer 2 (1000ms): publish_status()

执行模型: 单线程执行器
  - 两个定时器共享同一线程
  - 回调按时间先后顺序执行
  - 如果 receive_can_frames() 耗时过长，
    publish_status() 会延迟
```

#### 线程安全
```cpp
// SDO 读取使用互斥锁
std::mutex sdo_mutex_;

// 临界区保护
{
    std::lock_guard<std::mutex> lock(sdo_mutex_);
    sdo_response_received_ = true;
    sdo_read_value_ = data;
}
```

### 13.2 内存管理

#### 栈内存
```cpp
// CAN 帧在栈上分配
struct can_frame frame;  // 16 bytes on stack

// 快速，无需 malloc/free
// 函数返回后自动释放
```

#### 堆内存
```cpp
// ROS 2 消息使用智能指针
auto msg = std::make_shared<std_msgs::msg::Float32>();

// 自动引用计数
// 无订阅者时自动释放
```

#### 零拷贝优化
```cpp
// DDS 支持零拷贝（共享内存）
// 本地订阅者直接访问发布者内存
// 避免序列化/反序列化开销
```

### 13.3 实时性分析

#### 延迟来源

| 环节 | 典型延迟 | 影响因素 |
|------|---------|---------|
| **ROS 2 DDS** | 0.5-2 ms | 本地/远程, QoS设置 |
| **回调触发** | 0.1-0.5 ms | 执行器调度 |
| **应用逻辑** | 0.1-1 ms | 计算复杂度 |
| **SocketCAN** | 0.05-0.2 ms | 系统调用开销 |
| **CAN 传输** | 0.125-1 ms | 波特率, 帧长度 |
| **电机处理** | 1-5 ms | 驱动器响应时间 |

**总延迟**: 约 **2-10 ms**

#### CAN 传输时间计算
```
CAN Frame Transmission Time = Bit Time × Total Bits

Standard Frame (11-bit ID):
  Minimum: 47 bits (0 data bytes)
  Maximum: 111 bits (8 data bytes)

At 1 Mbps (Bit Time = 1 μs):
  Min: 47 μs = 0.047 ms
  Max: 111 μs = 0.111 ms

Bus Load:
  100 frames/s × 111 μs = 11.1 ms/s = 1.11% load
```

### 13.4 错误恢复策略

#### SDO 超时
```cpp
// 策略: 轮询 + 超时 (500ms)
int retry = 0;
const int max_retries = 50;

while (retry < max_retries && !response_received) {
    receive_can_frames();  // 尝试接收
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    retry++;
}

if (!response_received) {
    RCLCPP_WARN("SDO timeout");
    return 0;  // 返回默认值
}
```

#### 故障恢复
```cpp
// 检测故障
if (status_word & 0x0008) {
    // Step 1: 诊断
    check_and_clear_error();
    
    // Step 2: 复位
    set_control_word(CONTROL_FAULT_RESET);
    
    // Step 3: 重新使能
    enable_motor();
}
```

#### CAN Bus Off
```bash
# 自动重启配置
sudo ip link set can0 type can restart-ms 100

# 手动重启
sudo ip link set can0 type can restart
```

---

## 14. 性能分析与优化

### 14.1 性能指标

#### CPU 使用率
```bash
# 监控节点 CPU
top -p $(pgrep -f tower_crane)

# 典型值
Idle:       2-5%
Active:     10-20%
High Load:  30-50%
```

#### 内存占用
```bash
# 检查内存
ps aux | grep tower_crane

# 典型值
VSZ:  200-300 MB (虚拟内存)
RSS:  50-80 MB (物理内存)
```

#### CAN 总线负载
```bash
# 监控总线负载
candump -t d -n 1000 can0 | wc -l

# 计算负载
Load = (Frames/s × Avg_Frame_Size_bits) / Bitrate
```

### 14.2 优化建议

#### 减少 SDO 使用
```cpp
// ❌ 频繁 SDO 读取
for (int i = 0; i < 100; i++) {
    int32_t pos = read_sdo(OD_ACTUAL_POSITION, 0x00);  // 慢！
}

// ✅ 使用 PDO 订阅
// TPDO1 自动发送位置，无需主动查询
```

#### 批量操作
```cpp
// ❌ 逐个设置
write_sdo(0x6081, 0x00, velocity, 4);
write_sdo(0x6083, 0x00, accel, 4);
write_sdo(0x6084, 0x00, decel, 4);

// ✅ 一次性配置（初始化时）
set_profile_parameters(velocity, accel, decel);
```

#### 异步处理
```cpp
// ✅ 非阻塞读取
int flags = fcntl(can_socket_, F_GETFL, 0);
flags |= O_NONBLOCK;
fcntl(can_socket_, F_SETFL, flags);

// 快速返回，不等待
ssize_t n = read(can_socket_, &frame, sizeof(frame));
if (n < 0 && errno == EAGAIN) {
    // 无数据，继续
}
```

### 14.3 调试工具

#### CAN 总线监控
```bash
# 实时监控所有帧
candump can0

# 过滤特定 ID
candump can0,601:7FF

# 记录到文件
candump -l can0

# 播放记录
canplayer -I can0_log.log
```

#### 日志级别
```bash
# 调试模式
ros2 run crane_master tower_crane --ros-args --log-level debug

# 只看错误
ros2 run crane_master tower_crane --ros-args --log-level error
```

#### 性能分析
```bash
# CPU 分析
perf record -g ros2 run crane_master tower_crane
perf report

# 内存泄漏检测
valgrind --leak-check=full ros2 run crane_master tower_crane
```

---

## 附录 A: 完整配置示例

### A.1 CAN 接口配置脚本
```bash
#!/bin/bash
# setup_can.sh

CAN_INTERFACE="can0"
BITRATE="1000000"

# 加载模块
sudo modprobe can
sudo modprobe can_raw

# 配置接口
sudo ip link set ${CAN_INTERFACE} down
sudo ip link set ${CAN_INTERFACE} type can bitrate ${BITRATE} restart-ms 100
sudo ip link set ${CAN_INTERFACE} up

# 验证
ip -details link show ${CAN_INTERFACE}

echo "CAN interface ${CAN_INTERFACE} configured at ${BITRATE} bps"
```

### A.2 启动脚本
```bash
#!/bin/bash
# start_crane_master.sh

source /home/labcrane/appdata/ws_tower_crane/install/setup.bash

# 启动 Hook Motor (Node 1)
ros2 run crane_master tower_crane --ros-args \
  -r __ns:=/hoist \
  -p can_interface:=can0 \
  -p node_id:=1 \
  -p gear_ratio:=20.0 &

# 启动 Trolley Motor (Node 2)
ros2 run crane_master tower_crane --ros-args \
  -r __ns:=/trolley \
  -p can_interface:=can0 \
  -p node_id:=2 \
  -p gear_ratio:=10.0 &

# 启动 Slewing Motor (Node 3)
ros2 run crane_master tower_crane --ros-args \
  -r __ns:=/slewing \
  -p can_interface:=can0 \
  -p node_id:=3 \
  -p gear_ratio:=10.0 &

wait
```

---

## 附录 B: 故障排查流程图

```
问题: 电机不响应命令
    ↓
1. 检查 ROS 2 连接
   ros2 node list
    ├─ 节点未运行? → 启动节点
    └─ 节点运行 ↓
    
2. 检查话题通信
   ros2 topic echo /hoist/target_position
    ├─ 无数据? → 检查发布者
    └─ 有数据 ↓
    
3. 检查 CAN 接口
   ip link show can0
    ├─ DOWN? → sudo ip link set up can0
    └─ UP ↓
    
4. 监控 CAN 总线
   candump can0
    ├─ 无帧? → 检查应用层
    └─ 有帧 ↓
    
5. 检查 CAN ID
   candump can0 | grep 601
    ├─ 错误ID? → 检查 node_id 参数
    └─ 正确ID ↓
    
6. 检查电机状态
   ros2 topic echo /hoist/crane_status
    ├─ "故障"? → 调用 reset_crane 服务
    ├─ "禁止开启"? → 检查硬件使能信号
    └─ "操作已启用" ↓
    
7. 检查目标位置
   ros2 topic echo /hoist/crane_position
    ├─ 不变化? → 检查编码器连接
    └─ 变化但不到位? → 调整 PID 参数
```

---

## 附录 C: 参考资料

### 标准文档
- **CiA 301**: CANopen Application Layer and Communication Profile
- **CiA 402**: CANopen Device Profile for Drives and Motion Control
- **ISO 11898-1**: CAN Physical Layer Specification
- **RFC 7228**: Data Distribution Service (DDS) Specification

### 开源项目
- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **SocketCAN**: https://www.kernel.org/doc/html/latest/networking/can.html
- **can-utils**: https://github.com/linux-can/can-utils

### 学习资源
- **CANopen 入门**: https://www.can-cia.org/canopen/
- **Linux SocketCAN HOWTO**: https://www.kernel.org/doc/Documentation/networking/can.txt
- **ROS 2 设计**: https://design.ros2.org/

---

**文档版本**: 1.0  
**最后更新**: 2026-02-01  
**维护者**: crane_master Development Team

---

© 2026 Tower Crane Project. All rights reserved.
