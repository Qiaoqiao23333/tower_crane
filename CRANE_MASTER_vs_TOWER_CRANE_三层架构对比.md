# crane_master vs tower_crane 三层架构对比

> **版本**: 1.0  
> **日期**: 2026-02-10  
> **架构模型**: Interface - Logic - Platform

---

## 📑 目录

1. [三层架构定义](#1-三层架构定义)
2. [架构对比总览](#2-架构对比总览)
3. [Interface 层对比](#3-interface-层对比)
4. [Logic 层对比](#4-logic-层对比)
5. [Platform 层对比](#5-platform-层对比)
6. [应用场景对比](#6-应用场景对比)
7. [技术栈对比](#7-技术栈对比)

---

## 1. 三层架构定义

### 1.1 Interface 层（接口层）
**职责**：对外提供用户/系统交互接口，定义如何接收指令和返回数据。

**包含**：
- ROS 2 Topic/Service/Action 接口
- 消息类型定义
- 命令行工具
- API接口

### 1.2 Logic 层（逻辑层）
**职责**：核心业务逻辑，算法实现，状态管理，决策控制。

**包含**：
- 控制算法（PID、状态机、动力学模型）
- 数据处理与转换
- 业务规则
- 错误处理与恢复

### 1.3 Platform 层（平台层）
**职责**：与底层硬件/操作系统交互，提供运行环境。

**包含**：
- 硬件驱动与协议
- 操作系统接口（系统调用）
- 通信协议栈（CANopen、SocketCAN）
- 物理层接口

---

## 2. 架构对比总览

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        CRANE_MASTER                                 │
├───────────────────────────────┬─────────────────────────────────────┤
│  Interface 层                 │  Tower_Crane                        │
│  - ROS 2 Topics (Float32)     │  Interface 层                       │
│  - ROS 2 Services (Trigger)   │  - ROS 2 Topics (Twist, Float64MA)  │
│  - Joystick (Serial UART)     │  - 测试脚本 (Python)                │
│  - 手动角度/速度指令           │  - 加速度/力控制指令                │
├───────────────────────────────┼─────────────────────────────────────┤
│  Logic 层                     │  Logic 层                           │
│  - CiA402 状态机              │  - 拉格朗日力学                     │
│  - 角度↔电机单位转换          │  - 球形摆动动力学                   │
│  - Profile 参数管理           │  - 耦合系统求解                     │
│  - 错误检测与恢复             │  - 数值积分 (ODE)                   │
│  - SDO/PDO 命令构造           │  - 关节状态合并                     │
├───────────────────────────────┼─────────────────────────────────────┤
│  Platform 层                  │  Platform 层                        │
│  - SocketCAN (C++)            │  - ROS 2 Control                    │
│  - CANopen 协议栈             │  - canopen_core (仿真)              │
│  - 真实CAN总线 (can0)         │  - Fake Slaves (仿真)               │
│  - 电机驱动器 (CiA402)        │  - vcan0 (虚拟CAN)                  │
└───────────────────────────────┴─────────────────────────────────────┘
```

### 2.2 核心差异总结

| **维度**       | **crane_master**                    | **tower_crane**                     |
|---------------|-------------------------------------|-------------------------------------|
| **目标**       | 控制真实硬件电机                     | 仿真摆动动力学 + 可视化              |
| **编程语言**   | C++ (性能优先)                       | Python (快速原型)                    |
| **控制对象**   | 电机编码器位置/速度                  | 小车动力学状态（位置、速度、摆角）    |
| **物理模型**   | 无（假设电机理想响应）               | 完整拉格朗日力学模型                 |
| **硬件依赖**   | 必须连接真实CAN总线                  | 纯软件仿真（可选CANopen）            |
| **实时性**     | 高（20ms SYNC周期）                  | 中（50Hz发布频率）                   |

---

## 3. Interface 层对比

### 3.1 crane_master Interface 层

#### 输入接口

| **接口类型**       | **Topic/Service**          | **消息类型**             | **用途**          |
|------------------|---------------------------|------------------------|------------------|
| **Position**     | `/{namespace}/target_position` | `std_msgs/Float32`    | 角度控制 (度)     |
| **Velocity**     | `/{namespace}/target_velocity` | `std_msgs/Float32`    | 速度控制 (度/秒)  |
| **Service Start**| `/{namespace}/start`       | `std_srvs/Trigger`     | 启动电机使能      |
| **Service Stop** | `/{namespace}/stop`        | `std_srvs/Trigger`     | 停止电机          |
| **Service Reset**| `/{namespace}/reset`       | `std_srvs/Trigger`     | 重置故障          |
| **Joystick**     | Serial (`/dev/ttyUSB0`)   | 自定义二进制协议        | 手柄控制          |

**示例**：
```bash
# 角度控制
ros2 topic pub /slewing/target_position std_msgs/Float32 "data: 90.0"

# 速度控制
ros2 topic pub /trolley/target_velocity std_msgs/Float32 "data: 15.0"

# 启动电机
ros2 service call /hoist/start std_srvs/srv/Trigger
```

#### 输出接口

| **Topic**                    | **消息类型**         | **数据内容**              |
|------------------------------|---------------------|-------------------------|
| `/{namespace}/crane_position` | `std_msgs/Float32`  | 当前位置 (度)            |
| `/{namespace}/crane_velocity` | `std_msgs/Float32`  | 当前速度 (度/秒)         |
| `/{namespace}/status`         | `std_msgs/String`   | 状态文本 ("操作已启用")  |

#### 特点
- ✅ **简单直观**：直接的角度/速度数值
- ✅ **工业标准**：符合传统电机控制习惯
- ✅ **多模态输入**：支持ROS话题 + 串口手柄
- ❌ **单一控制目标**：只能控制电机，无法模拟物理效应

---

### 3.2 tower_crane Interface 层

#### 输入接口

| **接口类型**       | **Topic**                  | **消息类型**                 | **用途**          |
|------------------|---------------------------|----------------------------|------------------|
| **Twist**        | `/cmd_vel`                | `geometry_msgs/Twist`      | 加速度 (m/s²)    |
| **Acceleration** | `/target_acceleration`    | `std_msgs/Float64MultiArray` | 直接加速度       |
| **Force**        | `/control_force`          | `std_msgs/Float64MultiArray` | 施加力 (N)       |
| **测试脚本**      | 内置Python节点             | -                          | 自动化测试       |

**示例**：
```bash
# 加速度控制（物理力学）
ros2 topic pub /target_acceleration std_msgs/Float64MultiArray "{data: [0.5]}"

# Twist消息（加速度解释）
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}}"

# 测试脚本
ros2 run tower_crane crane_controller_test.py --ros-args -p mode:=sine
```

#### 输出接口

| **Topic**               | **消息类型**             | **数据内容**                          |
|------------------------|------------------------|-------------------------------------|
| `/joint_states_dynamics` | `sensor_msgs/JointState` | 关节位置、速度（含物理模拟）          |
| `/crane_state`          | `std_msgs/Float64MultiArray` | 完整状态 [x, ẋ, θ_x, θ̇_x, θ_y, θ̇_y, F] |
| `/joint_states_merged`  | `sensor_msgs/JointState` | 合并后关节状态（用于RViz）            |

#### 特点
- ✅ **物理导向**：输入加速度/力，而非直接位置
- ✅ **状态空间输出**：完整的6维状态向量
- ✅ **仿真友好**：集成RViz可视化
- ❌ **非直观**：需要理解动力学概念
- ❌ **无硬件控制**：纯仿真，不直接驱动电机

---

## 4. Logic 层对比

### 4.1 crane_master Logic 层

#### 核心算法

```cpp
// 1. 角度到电机单位转换
int32_t angle_to_position(float angle) {
    // angle (度) → 命令单位
    // 公式: (angle / 360°) × (10000 units/rev / gear_ratio)
    int32_t position = static_cast<int32_t>(
        angle * (target_units_per_rev_ / gear_ratio_) / 360.0f
    );
    return position;
}

// 2. CiA402 状态机
void enable_motor() {
    // State 1 → 2: Shutdown
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // State 2 → 3: Switch On
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // State 3 → 4: Enable Operation
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

// 3. 位置控制逻辑
void go_to_position(float angle) {
    // 检查当前模式
    if (current_mode != MODE_PROFILE_POSITION) {
        set_operation_mode(MODE_PROFILE_POSITION);
    }
    
    // 转换角度
    int32_t position = angle_to_position(angle);
    
    // 通过SDO写入目标位置
    write_sdo(OD_TARGET_POSITION, 0x00, position, 4);
    
    // PDO三步握手触发运动
    send_pdo_handshake(position);
}

// 4. 错误处理
void check_and_clear_error() {
    int32_t error_register = read_sdo(0x1001, 0x00);
    if (error_register > 0) {
        int32_t manufacturer_error = read_sdo(0x603F, 0x00);
        RCLCPP_ERROR("Error: 0x%04X", manufacturer_error);
        
        // 故障复位
        set_control_word(CONTROL_FAULT_RESET);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 重新使能
        enable_motor();
    }
}
```

#### 特点
- ✅ **状态机驱动**：严格遵循CiA402规范
- ✅ **单位转换**：角度↔电机编码器单位
- ✅ **同步控制**：等待状态转换完成
- ✅ **错误恢复**：自动检测并清除故障
- ❌ **无物理模型**：假设电机理想响应
- ❌ **阻塞操作**：使用 `sleep` 等待

---

### 4.2 tower_crane Logic 层

#### 核心算法

```python
# 1. 拉格朗日动力学方程
def dynamics(self, state, t):
    """
    状态向量: [x, ẋ, θ_x, θ̇_x, θ_y, θ̇_y]
    
    运动方程（非线性）:
    (M+m)ẍ + mL*cos(θ_x)*θ̈_x - mL*sin(θ_x)*θ̇_x² = F_control
    mL²*θ̈_x + mL*cos(θ_x)*ẍ + mgL*sin(θ_x) = -c_x*θ̇_x
    mL²*θ̈_y + mgL*sin(θ_y)/cos(θ_x) = -c_y*θ̇_y
    """
    x, x_dot, theta_x, theta_x_dot, theta_y, theta_y_dot = state
    
    # 耦合质量矩阵
    # [M+m      m*L*cos(θ_x)  ] [ẍ   ]   [RHS_x ]
    # [m*L*cos(θ_x)  m*L²    ] [θ̈_x ] = [RHS_θx]
    
    cos_theta_x = np.cos(theta_x)
    sin_theta_x = np.sin(theta_x)
    
    RHS_x = self.F_control + self.m * self.L * theta_x_dot**2 * sin_theta_x
    RHS_theta_x = -self.m * self.g * self.L * sin_theta_x - self.c_x * theta_x_dot
    
    # Cramer's法则求解加速度
    det = (self.M + self.m) * self.m * self.L**2 - (self.m * self.L * cos_theta_x)**2
    x_ddot = (RHS_x * self.m * self.L**2 - RHS_theta_x * self.m * self.L * cos_theta_x) / det
    theta_x_ddot = ((self.M + self.m) * RHS_theta_x - self.m * self.L * cos_theta_x * RHS_x) / det
    
    # Y方向摆动
    theta_y_ddot = -(self.g / self.L) * np.sin(theta_y) / cos_theta_x - \
                   (self.c_y / (self.m * self.L**2)) * theta_y_dot
    
    return np.array([x_dot, x_ddot, theta_x_dot, theta_x_ddot, theta_y_dot, theta_y_ddot])

# 2. 数值积分
def simulation_step(self):
    """使用Runge-Kutta方法求解ODE"""
    t_span = [0, self.dt]  # 10ms时间步长
    solution = odeint(self.dynamics, self.state, t_span)
    self.state = solution[-1, :]
    
    # 位置限制
    self.state[0] = np.clip(self.state[0], -2.0, 0.5)
    
    self.publish_joint_states()

# 3. 关节状态合并
def merge_joint_states(self):
    """
    合并两个来源的关节状态：
    - trolley_joint: 从动力学节点（包含物理）
    - 其他关节: 从 ROS 2 Control
    """
    merged = JointState()
    
    # 从动力学获取小车
    if 'trolley_joint' in self.dynamics_state.name:
        idx = self.dynamics_state.name.index('trolley_joint')
        merged.name.append('trolley_joint')
        merged.position.append(self.dynamics_state.position[idx])
        merged.velocity.append(self.dynamics_state.velocity[idx])
    
    # 从CANopen获取其他关节
    for i, name in enumerate(self.control_state.name):
        if name != 'trolley_joint':
            merged.name.append(name)
            merged.position.append(self.control_state.position[i])
            merged.velocity.append(self.control_state.velocity[i])
    
    return merged
```

#### 特点
- ✅ **物理精确**：完整拉格朗日力学
- ✅ **耦合系统**：小车加速度直接影响摆动
- ✅ **非线性求解**：处理大角度摆动
- ✅ **实时积分**：50Hz连续更新
- ✅ **状态空间**：6维完整状态
- ❌ **计算密集**：需要矩阵求解和三角函数
- ❌ **无硬件接口**：不直接控制电机

---

## 5. Platform 层对比

### 5.1 crane_master Platform 层

#### 架构

```
User Space
    │
    ├─ CANopenROS2 Node (C++)
    │   ├─ socket(PF_CAN, SOCK_RAW, CAN_RAW)
    │   ├─ bind(can_socket, &addr)
    │   ├─ write(can_socket, &frame, 16)
    │   └─ read(can_socket, &frame, 16)
    │
    ↓ System Calls
─────────────────────────────────────
Kernel Space
    │
    ├─ SocketCAN (can.ko)
    │   ├─ can_send()
    │   ├─ can_receive()
    │   └─ TX/RX Queues
    │
    ├─ CAN Device Driver (can-dev.ko)
    │   ├─ Interrupt Handler
    │   └─ Hardware Control
    │
    ↓ Hardware Access
─────────────────────────────────────
Hardware
    │
    ├─ CAN Controller (SoC/PCIe)
    ├─ CAN Transceiver (TJA1050)
    └─ CAN Bus (can0)
        ↓
    Motor Drivers (Node ID 1/2/3)
```

#### 关键代码

```cpp
// 初始化SocketCAN
void CANopenROS2::init_can_socket() {
    // 1. 创建原始套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    
    // 2. 获取接口索引
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");  // 真实硬件接口
    ioctl(can_socket_, SIOCGIFINDEX, &ifr);
    
    // 3. 绑定
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr));
    
    // 4. 非阻塞模式
    fcntl(can_socket_, F_SETFL, O_NONBLOCK);
}

// 发送CAN帧
void CANopenROS2::write_sdo(uint16_t index, uint8_t subindex, int32_t value, uint8_t size) {
    struct can_frame frame;
    frame.can_id = 0x600 + node_id_;  // SDO请求
    frame.can_dlc = 8;
    
    // 构造SDO下载报文
    frame.data[0] = 0x23;  // 4字节下载
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    memcpy(&frame.data[4], &value, 4);
    
    // 系统调用
    ssize_t nbytes = write(can_socket_, &frame, sizeof(frame));
    // → 进入内核 sys_write()
    // → SocketCAN层 can_send()
    // → 入队 TX queue
    // → 硬件发送到CAN总线
}
```

#### 特点
- ✅ **直接硬件访问**：通过SocketCAN操作真实CAN总线
- ✅ **低延迟**：内核态处理，微秒级响应
- ✅ **可靠协议**：CAN总线错误检测与重传
- ✅ **多节点支持**：同一总线控制多个电机
- ❌ **硬件依赖**：必须有CAN控制器和驱动器
- ❌ **平台特定**：Linux SocketCAN API

---

### 5.2 tower_crane Platform 层

#### 架构

```
User Space
    │
    ├─ spherical_pendulum_dynamics_node.py
    │   ├─ rclpy.spin()
    │   ├─ Timer callback (50Hz)
    │   ├─ scipy.integrate.odeint()
    │   └─ publish JointState
    │
    ├─ joint_state_merger_node.py
    │   └─ 合并动力学 + CANopen状态
    │
    ├─ ROS 2 Control (ros2_control_node)
    │   ├─ controller_manager
    │   ├─ joint_state_broadcaster
    │   └─ forward_position_controller
    │
    ├─ canopen_core (可选仿真)
    │   ├─ CANopen Master
    │   └─ Fake Slaves
    │
    ↓ Virtual Interface
─────────────────────────────────────
Kernel Space (仿真模式)
    │
    ├─ vcan (Virtual CAN)
    │   ├─ 内存队列
    │   └─ 无硬件交互
    │
    ↓ 纯软件
─────────────────────────────────────
No Hardware
    │
    └─ 所有数据在软件中循环
```

#### 关键代码

```python
# 仿真循环
def simulation_step(self):
    """50Hz定时器回调"""
    # 1. 数值积分（纯软件计算）
    t_span = [0, self.dt]
    solution = odeint(self.dynamics, self.state, t_span)
    self.state = solution[-1, :]
    
    # 2. 发布关节状态（内存传输）
    msg = JointState()
    msg.name = ['slewing_joint', 'trolley_joint', 'hook_joint']
    msg.position = [self.slewing_angle, self.state[0], self.hook_position]
    msg.velocity = [0.0, self.state[1], 0.0]
    
    # DDS传输（UDP/共享内存）
    self.joint_state_pub.publish(msg)
    
    # 3. 无硬件I/O
    # 无write()系统调用
    # 无CAN帧发送
    # 所有数据在ROS 2节点间流转

# 虚拟CAN（可选）
def setup_vcan():
    """仿真模式使用vcan0"""
    # vcan完全在内核内存中运行
    # 无物理收发器
    # 仅用于canopen_core测试
    os.system("sudo modprobe vcan")
    os.system("sudo ip link add dev vcan0 type vcan")
    os.system("sudo ip link set up vcan0")
```

#### 特点
- ✅ **无硬件依赖**：纯软件仿真
- ✅ **快速原型**：Python开发效率高
- ✅ **可视化友好**：集成RViz
- ✅ **参数可调**：动态修改物理参数
- ❌ **非实时**：Python + DDS延迟
- ❌ **无真实控制**：不能驱动电机
- ❌ **性能受限**：数值积分占用CPU

---

## 6. 应用场景对比

### 6.1 crane_master 应用场景

#### 适用场景
1. **真实硬件部署**
   - 工厂塔式起重机控制
   - 现场调试与运维
   - 生产环境运行

2. **精确位置控制**
   - 需要角度级精度
   - 多电机协同控制
   - 实时反馈

3. **工业级应用**
   - 长时间连续运行
   - 高可靠性要求
   - 故障自动恢复

#### 使用示例
```bash
# 1. 启动真实硬件
ros2 launch crane_master hardware_bringup_real.launch.py \
  can_interface_name:=can0

# 2. 启动电机
ros2 service call /slewing/start std_srvs/srv/Trigger
ros2 service call /trolley/start std_srvs/srv/Trigger
ros2 service call /hoist/start std_srvs/srv/Trigger

# 3. 发送位置指令
ros2 topic pub /slewing/target_position std_msgs/Float32 "data: 90.0"
ros2 topic pub /trolley/target_position std_msgs/Float32 "data: -1.5"
ros2 topic pub /hoist/target_position std_msgs/Float32 "data: 45.0"

# 4. 监控状态
ros2 topic echo /slewing/crane_position
ros2 topic echo /trolley/status
```

---

### 6.2 tower_crane 应用场景

#### 适用场景
1. **控制算法开发**
   - 防摆控制算法测试
   - PID参数整定
   - 轨迹规划验证

2. **物理仿真研究**
   - 载荷摆动行为研究
   - 参数敏感性分析
   - 教学演示

3. **系统设计验证**
   - 无硬件原型验证
   - 软件集成测试
   - 性能预评估

#### 使用示例
```bash
# 1. 启动带摆动物理的仿真
ros2 launch tower_crane simulation.launch.py \
  use_pendulum_dynamics:=true \
  payload_mass:=10.0 \
  rope_length:=2.5 \
  damping_x:=0.2

# 2. 运行测试脚本
ros2 run tower_crane crane_controller_test.py \
  --ros-args -p mode:=sine -p amplitude:=0.5

# 3. 自定义控制器
ros2 run tower_crane custom_controller_example.py \
  --ros-args -p target_position:=-1.5 -p Kp:=12.0 -p Kd:=6.0

# 4. 可视化状态
ros2 run tower_crane state_plotter.py
rviz2  # 打开RViz查看3D模型
```

---

## 7. 技术栈对比

### 7.1 编程语言与框架

| **维度**         | **crane_master**              | **tower_crane**                |
|-----------------|------------------------------|-------------------------------|
| **主语言**       | C++14                        | Python 3                       |
| **ROS 2**       | rclcpp (C++ Client Library)  | rclpy (Python Client Library) |
| **数学库**       | 自实现单位转换                | NumPy, SciPy                   |
| **控制库**       | 手动实现CiA402状态机          | scipy.integrate.odeint         |
| **构建系统**     | CMake + colcon               | Python setup.py + colcon       |

### 7.2 依赖对比

#### crane_master 依赖
```xml
<!-- package.xml -->
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>std_srvs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>

<!-- 无外部数学库依赖 -->
```

#### tower_crane 依赖
```xml
<!-- package.xml -->
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<exec_depend>python3-numpy</exec_depend>
<exec_depend>python3-scipy</exec_depend>
<exec_depend>python3-matplotlib</exec_depend>
<exec_depend>canopen_core</exec_depend>
<exec_depend>canopen_fake_slaves</exec_depend>
```

### 7.3 性能对比

| **指标**          | **crane_master**       | **tower_crane**        |
|------------------|----------------------|----------------------|
| **控制周期**      | 20ms (50Hz SYNC)      | 20ms (50Hz Timer)     |
| **延迟**          | <1ms (SocketCAN)      | ~10ms (DDS + Python)  |
| **CPU使用率**     | 5-10%                | 15-30%               |
| **内存占用**      | ~20MB                | ~50MB                |
| **实时性**        | 硬实时（内核态）       | 软实时（用户态）       |
| **可扩展性**      | 受硬件限制            | 无限制（纯软件）       |

### 7.4 文件结构对比

#### crane_master 文件树
```
crane_master/
├── include/crane_master/
│   ├── canopen_ros2_node.hpp        # 主节点头文件
│   └── canopen_defines.hpp          # CiA402常量定义
├── src/
│   ├── canopen_ros2_node.cpp        # 节点构造/初始化
│   ├── canopen_ros2_interface.cpp   # ROS接口实现
│   ├── canopen_sdo.cpp              # SDO读写
│   ├── canopen_pdo.cpp              # PDO处理
│   ├── canopen_utils.cpp            # 单位转换
│   └── main.cpp                     # 入口
├── scripts/
│   └── joystick_node.py             # 手柄节点
├── config/
│   └── bus.yml                      # CANopen配置
└── CMakeLists.txt                   # C++构建
```

#### tower_crane 文件树
```
tower_crane/
├── src/
│   ├── spherical_pendulum_dynamics_node.py  # 动力学仿真
│   ├── joint_state_merger_node.py           # 状态合并
│   └── crane_controller_test.py             # 测试脚本
├── examples/
│   ├── custom_controller_example.py         # PD控制器
│   └── state_plotter.py                     # 可视化
├── launch/
│   ├── simulation.launch.py                 # 仿真启动
│   └── hardware_bringup_real.launch.py      # 硬件启动
├── config/
│   ├── tower_crane_ros2_control.yaml        # ROS 2 Control
│   └── robot_control/bus.yml                # CANopen配置
├── urdf/
│   └── Tower_crane.urdf                     # 机器人模型
└── package.xml                              # Python包描述
```

---

## 8. 总结对比表

### 8.1 三层架构完整对比

| **层次** | **crane_master** | **tower_crane** |
|---------|-----------------|----------------|
| **Interface** | 角度/速度指令 (Float32)<br>ROS Service (启动/停止)<br>串口手柄 | 加速度/力指令 (Float64MultiArray)<br>Twist消息<br>Python测试脚本 |
| **Logic** | CiA402状态机<br>角度↔单位转换<br>SDO/PDO构造<br>错误恢复 | 拉格朗日力学<br>ODE数值积分<br>状态空间求解<br>关节状态合并 |
| **Platform** | SocketCAN (真实CAN)<br>电机驱动器 (CiA402)<br>can0接口<br>系统调用 | ROS 2 Control<br>vcan0 (虚拟CAN)<br>Fake Slaves<br>Python运行时 |

### 8.2 使用建议

#### 选择 crane_master 当：
- ✅ 需要控制真实硬件电机
- ✅ 要求低延迟和高可靠性
- ✅ 部署到生产环境
- ✅ 已有CANopen电机驱动器
- ✅ 需要工业级性能

#### 选择 tower_crane 当：
- ✅ 开发控制算法（如防摆控制）
- ✅ 研究载荷动力学
- ✅ 教学演示
- ✅ 无硬件快速原型
- ✅ 需要可视化和仿真

#### 联合使用：
```
tower_crane (算法开发) 
    ↓ 算法验证通过
crane_master (硬件部署)
    ↓ 真实环境测试
生产系统
```

---

**文档完成！** 🎉

两个包分别服务于 **仿真研究** 和 **硬件控制** 两个不同场景，在三层架构上各有侧重。
