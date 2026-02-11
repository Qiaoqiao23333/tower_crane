# Crane Master - Target_Position 完整数据流技术路线

## 目录
1. [概述](#概述)
2. [用户接口层](#1-用户接口层--指令输入)
3. [ROS 2 中间件层](#2-ros-2-中间件层--消息传输)
4. [应用逻辑层](#3-应用逻辑层--控制逻辑与状态机)
5. [CANopen 协议层](#4-canopen-协议层--sdo与pdo打包)
6. [Linux 内核层](#5-linux-内核层--系统调用)
7. [硬件物理层](#6-硬件物理层--can总线传输)
8. [完整数据流示例](#完整数据流示例)

---

## 概述

本文档详细描述 `crane_master` 包中 **Target_Position** 指令从用户输入到电机硬件执行的完整数据流程，涵盖六个技术层次：

```
┌─────────────────────────────────────────────────────────────────┐
│  用户接口层    │  ros2 topic pub /slewing/target_position     │
├─────────────────────────────────────────────────────────────────┤
│  ROS 2中间件层 │  DDS → Topic传输 → Callback触发              │
├─────────────────────────────────────────────────────────────────┤
│  应用逻辑层    │  角度→命令单位 + CiA402状态机控制           │
├─────────────────────────────────────────────────────────────────┤
│  CANopen协议层 │  SDO写入 + PDO握手 + SYNC同步                │
├─────────────────────────────────────────────────────────────────┤
│  Linux内核层   │  write() → SocketCAN → Kernel CAN Driver     │
├─────────────────────────────────────────────────────────────────┤
│  硬件物理层    │  CAN帧 → 差分电平信号 → 电机驱动器           │
└─────────────────────────────────────────────────────────────────┘
```

---

## 1. 用户接口层 — 指令输入

### 1.1 用户如何输入指令

用户通过 **ROS 2 Topic** 发布 Float32 消息来控制电机位置。

#### 方式一：命令行直接发布

```bash
# 控制回转电机移动到 90 度
ros2 topic pub /slewing/target_position std_msgs/msg/Float32 "data: 90.0" --once

# 控制小车电机移动到 180 度
ros2 topic pub /trolley/target_position std_msgs/msg/Float32 "data: 180.0" --once

# 控制吊钩电机移动到 45 度
ros2 topic pub /hoist/target_position std_msgs/msg/Float32 "data: 45.0" --once
```

#### 方式二：通过 Action Server（同步轨迹）

```bash
ros2 action send_goal /forward_position_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [90.0, 180.0, 45.0]
    time_from_start:
      sec: 2
"
```

### 1.2 指令格式

| **参数**     | **类型**             | **说明**                                    |
|-------------|---------------------|-------------------------------------------|
| Topic Name  | String              | `/slewing/target_position` (节点特定命名空间) |
| Message Type| `std_msgs/msg/Float32` | ROS 2 标准浮点数消息                        |
| Data Field  | float32             | 角度值（单位：度，°）                        |
| 数值范围    | -∞ ~ +∞             | 无硬编码限制，由电机驱动器内部限制           |

### 1.3 命名空间映射

| **电机名称** | **命名空间** | **CAN Node ID** | **减速比** |
|------------|------------|----------------|-----------|
| 回转电机     | /slewing   | 3              | 10.0      |
| 小车电机     | /trolley   | 2              | 10.0      |
| 吊钩电机     | /hoist     | 1              | 20.0      |

---

## 2. ROS 2 中间件层 — 消息传输

### 2.1 DDS 数据分发服务

ROS 2 使用 **DDS (Data Distribution Service)** 作为底层通信中间件。

```
User Command → DDS Publisher → Ethernet/Loopback → DDS Subscriber → Node Callback
```

### 2.2 Topic 传输路径

```cpp
// 文件：canopen_ros2_node.cpp 构造函数
position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/" + namespace_ + "/target_position",  // 例如：/slewing/target_position
    10,  // QoS队列深度
    std::bind(&CANopenROS2::position_callback, this, std::placeholders::_1)
);
```

**数据流物流过程：**

1. **发送方（Publisher）**：
   - 用户执行 `ros2 topic pub` 命令
   - ROS 2 客户端将 Float32 消息序列化
   - DDS 发布器广播到本地 DDS 域（默认 Domain ID 0）

2. **接收方（Subscriber）**：
   - `CANopenROS2` 节点订阅对应 Topic
   - DDS 订阅器接收消息并反序列化
   - 触发 `position_callback()` 回调函数

### 2.3 Callback 触发机制

```cpp
// 文件：canopen_ros2_interface.cpp
void CANopenROS2::position_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float angle = msg->data;  // 提取角度值（例如：90.0°）
    RCLCPP_INFO(this->get_logger(), "收到目标位置: %.2f°", angle);
    
    // 读取当前状态字
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    
    // 读取当前操作模式
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    
    // 调用应用逻辑层
    go_to_position(angle);
}
```

---

## 3. 应用逻辑层 — 控制逻辑与状态机

### 3.1 CiA402 状态机（DS402 Device Profile）

CANopen 电机遵循 **CiA402 标准状态机**，必须按顺序转换状态：

```
  ┌──────────────┐
  │ Not Ready to │
  │  Switch On   │
  └──────┬───────┘
         │ 自动进入
         ▼
  ┌──────────────┐   Shutdown    ┌──────────────┐
  │ Switch On    │◄──────────────┤ Ready to     │
  │  Disabled    │               │  Switch On   │
  └──────┬───────┘               └──────┬───────┘
         │ Shutdown                     │ Switch On
         ▼                              ▼
  ┌──────────────┐   Enable Op   ┌──────────────┐
  │   Switched   │◄──────────────┤   Switched   │
  │     On       │   (0x0007)    │     On       │
  └──────────────┘               └──────┬───────┘
                                        │ Enable Operation (0x000F)
                                        ▼
                                 ┌──────────────┐
                                 │  Operation   │
                                 │   Enabled    │ ← 电机可接受运动指令
                                 └──────────────┘
```

**关键控制字（Control Word）：**

| **控制字** | **16进制值** | **状态转换**                  |
|-----------|-------------|------------------------------|
| Shutdown  | 0x0006      | 进入 "Ready to Switch On"    |
| Switch On | 0x0007      | 进入 "Switched On"           |
| Enable Op | 0x000F      | 进入 "Operation Enabled"     |
| New Setpoint | 0x001F   | 触发新位置指令（位4置1）      |
| Fault Reset | 0x0080   | 清除故障状态                  |

### 3.2 角度到电机命令单位的转换

电机驱动器使用 **整数命令单位** 而非角度，需要根据减速比和分辨率转换。

#### 转换公式

```cpp
// 文件：canopen_utils.cpp
int32_t CANopenROS2::angle_to_position(float angle)
{
    // 公式：
    //   position = angle × (target_units_per_rev / gear_ratio) / 360°
    //
    // 例子（回转电机，减速比 10，分辨率 10000 units/rev）：
    //   90° → 90 × (10000 / 10) / 360 = 90 × 1000 / 360 = 250 units
    
    int32_t position = static_cast<int32_t>(angle * units_per_degree_);
    return position;
}
```

**参数说明：**
- `angle`：用户输入的角度（度）
- `target_units_per_rev_`：每圈输出轴的命令单位数（默认 10000）
- `gear_ratio_`：减速比（回转/小车：10.0，吊钩：20.0）
- `units_per_degree_`：缓存的转换系数 = `target_units_per_rev_ / gear_ratio_ / 360`

#### 实际数值示例

| **电机** | **角度输入** | **减速比** | **计算过程**              | **命令单位输出** |
|---------|------------|-----------|--------------------------|----------------|
| 回转     | 90°        | 10.0      | 90 × (10000/10)/360      | 250            |
| 小车     | 180°       | 10.0      | 180 × (10000/10)/360     | 500            |
| 吊钩     | 45°        | 20.0      | 45 × (10000/20)/360      | 62.5 → 62      |

### 3.3 位置控制流程（go_to_position）

```cpp
// 文件：canopen_motor_control.cpp
void CANopenROS2::go_to_position(float angle)
{
    RCLCPP_INFO(this->get_logger(), "移动到位置: %.2f°", angle);
    
    // 步骤1：角度转换为命令单位
    int32_t position = angle_to_position(angle);  // 例如：90° → 250 units
    RCLCPP_INFO(this->get_logger(), "目标位置命令单位: %d", position);
    
    // 步骤2：使用 SDO 写入目标位置（一次性设置）
    write_sdo(OD_TARGET_POSITION, 0x00, position, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 步骤3：使用 PDO 进行控制字握手（避免与SDO混用导致状态机混乱）
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;  // 例如：0x200 + 3 = 0x203（回转电机）
    frame.can_dlc = 6;  // 2 bytes control word + 4 bytes position
    
    // 3a. 发送 Enable Operation (0x000F) + 目标位置
    frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;         // 低字节：0x0F
    frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;  // 高字节：0x00
    frame.data[2] = position & 0xFF;                         // 位置第1字节
    frame.data[3] = (position >> 8) & 0xFF;                  // 位置第2字节
    frame.data[4] = (position >> 16) & 0xFF;                 // 位置第3字节
    frame.data[5] = (position >> 24) & 0xFF;                 // 位置第4字节
    
    write(can_socket_, &frame, sizeof(struct can_frame));
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // 3b. 发送 New Setpoint (0x001F) - 上升沿触发新位置
    frame.data[0] = (CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT) & 0xFF;
    write(can_socket_, &frame, sizeof(struct can_frame));
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // 3c. 恢复 Enable Operation (0x000F) - 完成握手（下降沿）
    frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
    write(can_socket_, &frame, sizeof(struct can_frame));
    send_sync_frame();
    
    // 步骤4：监控目标位置到达（读取状态字 bit 10）
    int retry = 0;
    while (retry < 50) {  // 最多等待 10 秒
        int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
        
        // 检查目标到达位（bit 10，0x0400）
        if (status_word & 0x0400) {
            RCLCPP_INFO(this->get_logger(), "目标位置已到达");
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        retry++;
    }
}
```

---

## 4. CANopen 协议层 — SDO与PDO打包

CANopen 通信主要使用两种协议：**SDO（服务数据对象）** 和 **PDO（过程数据对象）**。

### 4.1 SDO（Service Data Object）— 配置和读取

SDO 用于 **非实时通信**，如配置参数、读取状态。

#### SDO 写入（下载）

```cpp
// 文件：canopen_can_communication.cpp
void CANopenROS2::write_sdo(uint16_t index, uint8_t subindex, int32_t data, uint8_t size)
{
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;  // 0x600 + node_id（例如：0x603）
    frame.can_dlc = 8;  // SDO 固定 8 字节
    
    // 命令字节（CCS - Client Command Specifier）
    uint8_t command = 0x22;  // 下载请求
    if (size == 1)      command |= 0x0F;  // 1字节加速下载
    else if (size == 2) command |= 0x0B;  // 2字节加速下载
    else if (size == 4) command |= 0x03;  // 4字节加速下载
    
    frame.data[0] = command;                    // 命令字节
    frame.data[1] = index & 0xFF;               // 索引低字节
    frame.data[2] = (index >> 8) & 0xFF;        // 索引高字节
    frame.data[3] = subindex;                   // 子索引
    frame.data[4] = data & 0xFF;                // 数据第1字节
    frame.data[5] = (data >> 8) & 0xFF;         // 数据第2字节
    frame.data[6] = (data >> 16) & 0xFF;        // 数据第3字节
    frame.data[7] = (data >> 24) & 0xFF;        // 数据第4字节
    
    write(can_socket_, &frame, sizeof(struct can_frame));
}
```

**示例：写入目标位置 90° → 250 units**

```
CAN ID: 0x603 (RSDO + Node ID 3)
DLC: 8
Data: [0x23, 0x7A, 0x60, 0x00, 0xFA, 0x00, 0x00, 0x00]
      │     │     │     │     └─┴─┴─┴── 数据：250 (0x000000FA)
      │     │     │     └── 子索引：0x00
      │     └─────┴── 索引：0x607A (Target Position)
      └── 命令：0x23 (4字节加速下载)
```

#### SDO 读取（上传）

```cpp
int32_t CANopenROS2::read_sdo(uint16_t index, uint8_t subindex)
{
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;  // 0x600 + node_id
    frame.can_dlc = 8;
    frame.data[0] = 0x40;  // 上传请求
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    // 标记期望的响应
    expected_sdo_index_ = index;
    expected_sdo_subindex_ = subindex;
    sdo_response_received_ = false;
    
    write(can_socket_, &frame, sizeof(struct can_frame));
    
    // 等待响应（COB-ID: 0x580 + node_id）
    int retry = 0;
    while (retry < 50 && !sdo_response_received_) {
        receive_can_frames();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        retry++;
    }
    
    return sdo_read_value_;
}
```

**示例：读取实际位置**

```
请求：
  CAN ID: 0x603
  Data: [0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
        │     │     │     │
        │     └─────┴── 索引：0x6064 (Actual Position)
        └── 命令：0x40 (上传请求)

响应：
  CAN ID: 0x583 (TSDO + Node ID 3)
  Data: [0x43, 0x64, 0x60, 0x00, 0xFA, 0x00, 0x00, 0x00]
        │     │     │     │     └─┴─┴─┴── 数据：250 units
        │     └─────┴── 索引确认
        └── 命令：0x43 (4字节加速上传响应)
```

### 4.2 PDO（Process Data Object）— 实时控制

PDO 用于 **实时通信**，低延迟、高频率。

#### PDO 映射配置

在初始化阶段配置 PDO 映射（将对象字典条目映射到 PDO）：

```cpp
// 文件：canopen_motor_control.cpp - configure_pdo()

// RxPDO1 配置（主站→从站，控制命令）
void CANopenROS2::configure_pdo()
{
    // 1. 禁用 RxPDO1
    uint32_t rxpdo1_cob_id = COB_RPDO1 + node_id_;  // 0x200 + 3 = 0x203
    write_sdo(0x1400, 0x01, rxpdo1_cob_id | 0x80000000, 4);  // 最高位置1禁用
    
    // 2. 清除映射
    write_sdo(0x1600, 0x00, 0x00, 1);
    
    // 3. 映射控制字（0x6040，16位）
    write_sdo(0x1600, 0x01, 0x60400010, 4);
    
    // 4. 映射目标位置（0x607A，32位）
    write_sdo(0x1600, 0x02, 0x607A0020, 4);
    
    // 5. 设置映射对象数量
    write_sdo(0x1600, 0x00, 0x02, 1);
    
    // 6. 启用 RxPDO1
    write_sdo(0x1400, 0x01, rxpdo1_cob_id, 4);
}
```

**PDO 映射解析：**

```
0x60400010 = 0x6040 (索引) + 00 (子索引) + 10 (16位)
0x607A0020 = 0x607A (索引) + 00 (子索引) + 20 (32位)
```

#### PDO 发送（RxPDO1 - 位置控制）

```cpp
void CANopenROS2::set_control_word(uint16_t control_word)
{
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;  // 例如：0x203（回转电机）
    frame.can_dlc = 6;  // 2 bytes control + 4 bytes position
    
    frame.data[0] = control_word & 0xFF;         // 控制字低字节
    frame.data[1] = (control_word >> 8) & 0xFF;  // 控制字高字节
    frame.data[2] = position & 0xFF;             // 位置字节 1
    frame.data[3] = (position >> 8) & 0xFF;      // 位置字节 2
    frame.data[4] = (position >> 16) & 0xFF;     // 位置字节 3
    frame.data[5] = (position >> 24) & 0xFF;     // 位置字节 4
    
    write(can_socket_, &frame, sizeof(struct can_frame));
}
```

**示例：发送位置 250 units + 控制字 0x000F**

```
CAN ID: 0x203 (RxPDO1 for Node 3)
DLC: 6
Data: [0x0F, 0x00, 0xFA, 0x00, 0x00, 0x00]
      │     │     └─┴─┴─┴── 位置：250 (0x000000FA)
      └─────┴── 控制字：0x000F (Enable Operation)
```

### 4.3 SYNC 同步帧

**SYNC 帧**用于同步多个电机的运动，确保所有电机同时响应指令。

```cpp
// 文件：canopen_can_communication.cpp
void CANopenROS2::send_sync_frame()
{
    struct can_frame frame;
    frame.can_id = COB_SYNC;  // 0x080（固定）
    frame.can_dlc = 0;        // 无数据负载
    
    write(can_socket_, &frame, sizeof(struct can_frame));
    RCLCPP_DEBUG(this->get_logger(), "同步帧已发送");
}
```

**SYNC 帧格式：**

```
CAN ID: 0x080
DLC: 0
Data: (无)
```

**SYNC 工作机制：**

```
时间线：
t=0ms   → 发送 RxPDO1（回转电机，位置 250）
t=1ms   → 发送 RxPDO1（小车电机，位置 500）
t=2ms   → 发送 RxPDO1（吊钩电机，位置 62）
t=5ms   → 发送 SYNC (0x080)  ← 所有电机同时开始运动
```

### 4.4 CANopen 对象字典（Object Dictionary）

电机驱动器的参数存储在 **对象字典** 中，使用 **索引（Index）** 和 **子索引（Subindex）** 访问。

| **对象名称**       | **索引** | **子索引** | **数据类型** | **访问权限** | **描述**                  |
|-------------------|---------|-----------|------------|-------------|--------------------------|
| Control Word      | 0x6040  | 0x00      | UINT16     | RW          | 控制字（状态机命令）        |
| Status Word       | 0x6041  | 0x00      | UINT16     | RO          | 状态字（电机状态）          |
| Operation Mode    | 0x6060  | 0x00      | INT8       | RW          | 操作模式（位置/速度/扭矩）  |
| Target Position   | 0x607A  | 0x00      | INT32      | RW          | 目标位置（命令单位）        |
| Actual Position   | 0x6064  | 0x00      | INT32      | RO          | 实际位置（命令单位）        |
| Profile Velocity  | 0x6081  | 0x00      | UINT32     | RW          | 轮廓速度（命令单位/秒）     |
| Profile Accel     | 0x6083  | 0x00      | UINT32     | RW          | 轮廓加速度（命令单位/秒²）  |
| Profile Decel     | 0x6084  | 0x00      | UINT32     | RW          | 轮廓减速度（命令单位/秒²）  |

---

## 5. Linux 内核层 — 系统调用

### 5.1 SocketCAN 架构

Linux 内核使用 **SocketCAN** 框架将 CAN 总线抽象为网络接口。

```
┌─────────────────────────────────────────────────────────┐
│  用户空间 (crane_master)                                │
│  ├─ write(can_socket, &frame, sizeof(frame))          │
│  └─ read(can_socket, &frame, sizeof(frame))           │
└────────────────┬────────────────────────────────────────┘
                 │ System Call
                 ▼
┌─────────────────────────────────────────────────────────┐
│  Linux 内核 (Kernel Space)                              │
│  ├─ SocketCAN 核心 (af_can.c)                          │
│  ├─ CAN 设备驱动 (can_dev.c)                           │
│  └─ USB-CAN 驱动 (usb_8dev.c, peak_usb.c, etc.)       │
└────────────────┬────────────────────────────────────────┘
                 │ USB/SPI/GPIO
                 ▼
┌─────────────────────────────────────────────────────────┐
│  硬件 CAN 控制器 (MCP2515, USB-CAN 适配器)              │
└─────────────────────────────────────────────────────────┘
```

### 5.2 CAN Socket 初始化

```cpp
// 文件：canopen_can_communication.cpp
void CANopenROS2::init_can_socket()
{
    // 1. 创建 CAN 套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法创建CAN套接字");
        return;
    }
    
    // 2. 获取 CAN 接口索引
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());  // 例如："can0"
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法获取CAN接口索引");
        close(can_socket_);
        return;
    }
    
    // 3. 绑定套接字到 CAN 接口
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法绑定CAN套接字");
        close(can_socket_);
        return;
    }
    
    // 4. 设置非阻塞模式（可选）
    int flags = fcntl(can_socket_, F_GETFL, 0);
    fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
    
    RCLCPP_INFO(this->get_logger(), "CAN套接字初始化成功");
}
```

### 5.3 write() 系统调用

当调用 `write()` 发送 CAN 帧时，数据经过以下路径：

```cpp
// 用户空间
struct can_frame frame;
frame.can_id = 0x203;
frame.can_dlc = 6;
frame.data[0] = 0x0F;
// ... 填充数据 ...

ssize_t bytes_written = write(can_socket_, &frame, sizeof(struct can_frame));
```

**内核处理流程：**

1. **系统调用入口**：`write()` → `sys_write()` → `sock_write_iter()`
2. **SocketCAN 处理**：`can_send()` → `can_send_check()`
3. **设备驱动**：`can_put_echo_skb()` → `netdev_start_xmit()`
4. **硬件发送**：USB-CAN 驱动 → USB 传输 → CAN 控制器发送

### 5.4 CAN 帧内核结构

```c
// 内核定义：include/linux/can.h
struct can_frame {
    canid_t can_id;  // 32位 CAN ID + 标志位
    __u8    can_dlc; // 数据长度（0-8）
    __u8    __pad;   // 填充
    __u8    __res0;  // 保留
    __u8    __res1;  // 保留
    __u8    data[8]; // 数据字节
};
```

**can_id 字段解析：**

```
Bit 31: ERR (错误帧)
Bit 30: RTR (远程传输请求)
Bit 29: EFF (扩展帧格式)
Bit 28-11: 保留
Bit 10-0: CAN ID (标准帧11位)
```

---

## 6. 硬件物理层 — CAN总线传输

### 6.1 CAN 总线电气特性

CAN 总线使用 **差分信号** 传输，具有高抗干扰能力。

```
CAN_H ──────┐        ┌──── 高电平：3.5V
             │ 差分电压│
CAN_L ──────┘ (2V)   └──── 低电平：1.5V

显性位 (Dominant, 0)：差分电压 ≈ 2V
隐性位 (Recessive, 1)：差分电压 ≈ 0V
```

### 6.2 CAN 帧格式（标准帧）

```
┌───┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬───┐
│SOF│ ID │RTR │IDE │R0  │DLC │DATA│CRC │ACK │EOF │IFS │...│
└───┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴───┘
 1   11   1   1   1   4   0-64  15  2   7   3
位   位   位  位  位  位   位    位  位  位  位
```

**字段说明：**
- **SOF (Start of Frame)**：帧起始位（1位，显性）
- **ID**：仲裁字段（11位标准ID，29位扩展ID）
- **RTR**：远程传输请求（1位）
- **IDE**：标识符扩展位（1位，标准帧为0）
- **R0**：保留位（1位）
- **DLC**：数据长度码（4位，0-8字节）
- **DATA**：数据字段（0-8字节）
- **CRC**：循环冗余校验（15位 + 1位界定符）
- **ACK**：应答位（2位：ACK Slot + ACK Delimiter）
- **EOF**：帧结束（7位隐性）
- **IFS**：帧间隔（3位隐性）

### 6.3 实际 CAN 帧示例

**示例：RxPDO1 发送位置 250 units + 控制字 0x000F**

```
┌──────────────────────────────────────────────────────────────┐
│ SOF │ CAN ID (0x203) │ RTR │ IDE │ R0 │ DLC (6) │ DATA      │
├─────┼────────────────┼─────┼─────┼────┼─────────┼───────────┤
│  0  │ 10000000011   │  0  │  0  │ 0  │  0110   │ 48 bits   │
└─────┴────────────────┴─────┴─────┴────┴─────────┴───────────┘
         (0x203 = 515)                    (6 bytes)

DATA 字段（十六进制）：
  Byte 0: 0x0F  (控制字低字节)
  Byte 1: 0x00  (控制字高字节)
  Byte 2: 0xFA  (位置字节1)
  Byte 3: 0x00  (位置字节2)
  Byte 4: 0x00  (位置字节3)
  Byte 5: 0x00  (位置字节4)
```

**完整二进制表示（简化）：**

```
0 10000000011 0 0 0 0110 00001111 00000000 11111010 00000000 00000000 00000000 [CRC] 00 1111111 111
│ └─────┬─────┘ │ │ │ └┬─┘ └──┬───┘ └──┬───┘ └──┬───┘ └──┬───┘ └──┬───┘ └──┬───┘      ││ └──┬───┘
SOF    ID       R I R DLC   Byte0    Byte1    Byte2    Byte3    Byte4    Byte5       ACK  EOF
                T D 0                                                                   
                R E
```

### 6.4 波特率与位时序

CAN 总线常用波特率：**1 Mbps**（本项目配置）

```bash
# 配置 can0 接口
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

**位时序计算（1 Mbps）：**

```
1 位时间 = 1 / 1,000,000 = 1 微秒 (μs)

每一位分为 4 段：
  - SYNC_SEG：同步段（1 TQ）
  - PROP_SEG：传播段（1-8 TQ）
  - PHASE_SEG1：相位缓冲段1（1-8 TQ）
  - PHASE_SEG2：相位缓冲段2（1-8 TQ）

TQ (Time Quantum) = 位时间 / (SYNC_SEG + PROP_SEG + PHASE_SEG1 + PHASE_SEG2)
```

### 6.5 物理连接拓扑

```
┌─────────────┐         ┌─────────────┐         ┌─────────────┐
│ 计算机      │         │ 回转电机     │         │ 小车电机     │
│ (crane_master)        │ (Node ID 3) │         │ (Node ID 2) │
│             │         │             │         │             │
│   USB-CAN   │         │  CAN 接口   │         │  CAN 接口   │
└──────┬──────┘         └──────┬──────┘         └──────┬──────┘
       │                       │                       │
       │ CAN_H ════════════════╪═══════════════════════╪═══ 120Ω
       │                       │                       │
       │ CAN_L ════════════════╪═══════════════════════╪═══ 120Ω
       │                       │                       │
       └───────────────────────┴───────────────────────┘
                              CAN 总线
                           (双绞线，终端电阻 120Ω)
```

**关键要点：**
- **终端电阻**：总线两端各接一个 120Ω 电阻
- **双绞线**：降低电磁干扰
- **总线拓扑**：所有节点并联连接
- **最大长度**：40m @ 1 Mbps（速度越高，距离越短）

---

## 完整数据流示例

### 场景：用户发送命令控制回转电机移动到 90°

#### 步骤详解

| **层次** | **操作** | **细节** |
|---------|---------|---------|
| **用户输入** | `ros2 topic pub /slewing/target_position std_msgs/msg/Float32 "data: 90.0"` | 用户在终端输入命令 |
| **ROS 2 中间件** | DDS 发布消息 → `position_callback()` 被触发 | Float32 消息传输，角度值 90.0 |
| **应用逻辑** | 调用 `go_to_position(90.0)` | |
| | ├─ 角度转换：`90.0° → 250 units` | 公式：90 × (10000/10)/360 = 250 |
| | ├─ 读取当前状态字：`read_sdo(0x6041, 0x00)` | 确认电机在 "Operation Enabled" 状态 |
| | ├─ SDO 写入目标位置：`write_sdo(0x607A, 0x00, 250, 4)` | 配置目标位置 |
| | ├─ PDO 握手（Step 1）：发送控制字 0x000F + 位置 250 | RxPDO1 (CAN ID 0x203) |
| | ├─ 发送 SYNC 帧 (0x080) | 同步触发 |
| | ├─ PDO 握手（Step 2）：发送控制字 0x001F + 位置 250 | 新设定点上升沿 |
| | ├─ 发送 SYNC 帧 (0x080) | |
| | ├─ PDO 握手（Step 3）：发送控制字 0x000F + 位置 250 | 握手下降沿 |
| | └─ 发送 SYNC 帧 (0x080) | 电机开始运动 |
| **CANopen 协议** | SDO 帧打包 | |
| | ├─ CAN ID: 0x603 (RSDO + Node ID 3) | |
| | ├─ DLC: 8 | |
| | └─ Data: [0x23, 0x7A, 0x60, 0x00, 0xFA, 0x00, 0x00, 0x00] | |
| | PDO 帧打包（握手 Step 1） | |
| | ├─ CAN ID: 0x203 (RxPDO1 + Node ID 3) | |
| | ├─ DLC: 6 | |
| | └─ Data: [0x0F, 0x00, 0xFA, 0x00, 0x00, 0x00] | |
| | SYNC 帧打包 | |
| | ├─ CAN ID: 0x080 | |
| | └─ DLC: 0 | |
| **Linux 内核** | `write(can_socket_, &frame, 16)` | 系统调用 |
| | ├─ SocketCAN 核心处理 | af_can.c |
| | ├─ CAN 设备驱动调度 | can_dev.c |
| | └─ USB-CAN 驱动发送 | usb_8dev.c / peak_usb.c |
| **硬件物理层** | USB-CAN 适配器发送 | |
| | ├─ CAN 控制器编码帧 | |
| | ├─ 差分信号输出（CAN_H/CAN_L） | 显性位 0V，隐性位 2V |
| | └─ 总线传输到电机驱动器 | 双绞线，120Ω 终端电阻 |
| **电机驱动器** | 接收 CAN 帧 | |
| | ├─ 解码 RxPDO1：控制字 0x000F，位置 250 | |
| | ├─ 等待 SYNC 帧 (0x080) | |
| | ├─ 执行位置指令：输出轴转动到 250 units | |
| | └─ 反馈状态字 (TxPDO1)：位 10 置 1（目标到达） | |

---

### 时序图

```
时间线（毫秒）   crane_master           SocketCAN          CAN 总线        电机驱动器
    │                                                       │                │
t=0 │ position_callback(90.0)                              │                │
    │ ├─ angle_to_position(90.0) → 250                     │                │
    │ └─ go_to_position(250)                               │                │
    │                                                       │                │
t=5 │ write_sdo(0x607A, 250) ────────┐                     │                │
    │                                │                     │                │
t=6 │                                └─► CAN 帧 0x603 ─────┼───────────────►│
    │                                                       │  SDO 写入       │
    │                                                       │  目标位置 250   │
    │                                                       │                │
t=10│ set_control_word(0x000F) ──────┐                     │                │
    │   + position(250)              │                     │                │
t=11│                                └─► CAN 帧 0x203 ─────┼───────────────►│
    │                                                       │  RxPDO1        │
    │                                                       │  (等待SYNC)    │
    │                                                       │                │
t=12│ send_sync_frame() ─────────────┐                     │                │
t=13│                                └─► CAN 帧 0x080 ─────┼───────────────►│
    │                                                       │  SYNC 触发     │
    │                                                       │                │
t=15│                                                       │                ├─ 开始运动
    │                                                       │                │   (电机加速)
    │                                                       │                │
    │ [等待运动完成...]                                      │                │
    │                                                       │                │
t=2000                                                      │                ├─ 到达目标
    │                                                       │                │   (250 units)
    │                                                       │                │
t=2001                            CAN 帧 0x183 ◄───────────┼────────────────┤
    │ receive_can_frames() ◄────────┘                      │  TxPDO1        │
    │ └─ status_word_ = 0x0427                             │  (状态字 bit10=1)
    │    (bit 10 置1，目标到达)                             │                │
    │                                                       │                │
t=2002 RCLCPP_INFO("目标位置已到达")                         │                │
    │                                                       │                │
```

---

## 总结

### 数据流关键路径

```
用户角度 (90°)
    ↓ ROS 2 DDS 传输
应用层换算 (250 units)
    ↓ CiA402 状态机控制
CANopen SDO/PDO 打包
    ↓ Linux write() 系统调用
SocketCAN 内核处理
    ↓ USB-CAN 驱动
CAN 总线差分信号
    ↓ 电机驱动器解码
电机物理运动 (输出轴旋转)
```

### 技术要点

1. **用户接口层**：通过 ROS 2 Topic 发布 Float32 消息
2. **ROS 2 中间件**：DDS 负责消息分发，触发回调函数
3. **应用逻辑层**：
   - 角度转换为命令单位（考虑减速比和分辨率）
   - CiA402 状态机严格按序转换（Shutdown → Switch On → Enable Operation）
   - 使用 SDO 配置 + PDO 实时控制的混合策略
4. **CANopen 协议层**：
   - SDO：配置参数、读取状态（非实时）
   - PDO：实时运动控制（低延迟）
   - SYNC：多电机同步触发
5. **Linux 内核层**：SocketCAN 将 CAN 总线抽象为网络接口，通过 `write()` 发送帧
6. **硬件物理层**：CAN 总线使用差分信号，抗干扰能力强，支持多主通信

### 性能指标

| **指标** | **数值** | **说明** |
|---------|---------|---------|
| 命令延迟 | < 5 ms | 从 ROS 2 Topic 到 CAN 帧发送 |
| CAN 波特率 | 1 Mbps | 标准工业配置 |
| 位置分辨率 | 0.036° | 10000 units/rev，减速比 10 |
| SYNC 周期 | 可配置 | 默认 10 ms (100 Hz) |
| SDO 超时 | 500 ms | 读取 SDO 最大等待时间 |
| 电机响应 | < 20 ms | 从接收 SYNC 到开始运动 |

---

**文档版本**：v1.0  
**创建日期**：2026-02-03  
**作者**：Crane Master Team  
**适用包**：crane_master (ROS 2 Humble)
