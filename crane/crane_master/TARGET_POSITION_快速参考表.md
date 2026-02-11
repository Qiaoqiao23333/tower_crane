# Target_Position 快速参考表

## 📋 一页纸速查手册

---

## 1️⃣ 六层架构速览

| **层次** | **技术栈** | **关键函数/工具** | **延迟** |
|---------|-----------|----------------|---------|
| **用户接口层** | ROS 2 Topic | `ros2 topic pub` | 0ms |
| **ROS 2中间件** | DDS | `create_subscription()` | < 5ms |
| **应用逻辑层** | C++ | `go_to_position()`, `angle_to_position()` | < 1ms |
| **CANopen协议** | CiA 301/402 | `write_sdo()`, PDO, SYNC | < 1ms |
| **Linux内核** | SocketCAN | `write()`, `read()` | < 0.5ms |
| **硬件物理层** | CAN Bus | USB-CAN 适配器, 差分信号 | < 0.1ms |

**总延迟**：< 10 ms （从用户输入到CAN帧发送）

---

## 2️⃣ 节点与电机映射表

| **电机名称** | **ROS 2 命名空间** | **CAN Node ID** | **减速比** | **分辨率** |
|------------|--------------------|----------------|-----------|-----------|
| 回转电机 (Slewing) | `/slewing` | 3 | 10.0 | 0.036°/unit |
| 小车电机 (Trolley) | `/trolley` | 2 | 10.0 | 0.036°/unit |
| 吊钩电机 (Hoist) | `/hoist` | 1 | 20.0 | 0.018°/unit |

---

## 3️⃣ ROS 2 接口速查

### Topics

| **Topic** | **消息类型** | **方向** | **描述** |
|----------|------------|---------|---------|
| `/{namespace}/target_position` | `std_msgs/Float32` | 订阅 (Rx) | 设置目标位置（度） |
| `/{namespace}/target_velocity` | `std_msgs/Float32` | 订阅 (Rx) | 设置目标速度（度/秒） |
| `/{namespace}/crane_position` | `std_msgs/Float32` | 发布 (Tx) | 当前位置（度） |
| `/{namespace}/crane_velocity` | `std_msgs/Float32` | 发布 (Tx) | 当前速度（度/秒） |
| `/{namespace}/crane_status` | `std_msgs/String` | 发布 (Tx) | 电机状态文本 |

### Services

| **Service** | **类型** | **描述** |
|------------|---------|---------|
| `/{namespace}/start_crane` | `std_srvs/Trigger` | 启动电机（初始化） |
| `/{namespace}/stop_crane` | `std_srvs/Trigger` | 停止电机 |
| `/{namespace}/reset_crane` | `std_srvs/Trigger` | 重置电机（NMT Reset） |
| `/{namespace}/set_crane_mode` | `std_srvs/SetBool` | 切换模式（True=位置，False=速度） |

### Action Server

| **Action** | **类型** | **描述** |
|-----------|---------|---------|
| `/forward_position_controller/follow_joint_trajectory` | `FollowJointTrajectory` | 多关节同步轨迹跟踪 |

---

## 4️⃣ CANopen 对象字典速查

### 常用对象

| **对象名称** | **索引** | **子索引** | **类型** | **访问** | **描述** |
|------------|---------|-----------|---------|---------|---------|
| Control Word | 0x6040 | 0x00 | UINT16 | RW | 控制字（状态机命令） |
| Status Word | 0x6041 | 0x00 | UINT16 | RO | 状态字（电机状态） |
| Operation Mode | 0x6060 | 0x00 | INT8 | RW | 操作模式 |
| Op Mode Display | 0x6061 | 0x00 | INT8 | RO | 当前操作模式 |
| Target Position | 0x607A | 0x00 | INT32 | RW | 目标位置 |
| Actual Position | 0x6064 | 0x00 | INT32 | RO | 实际位置 |
| Profile Velocity | 0x6081 | 0x00 | UINT32 | RW | 轮廓速度 |
| Profile Accel | 0x6083 | 0x00 | UINT32 | RW | 轮廓加速度 |
| Profile Decel | 0x6084 | 0x00 | UINT32 | RW | 轮廓减速度 |
| Target Velocity | 0x60FF | 0x00 | INT32 | RW | 目标速度 |

---

## 5️⃣ CiA402 控制字速查

### 状态机转换

| **控制字** | **十六进制** | **二进制** | **状态转换** |
|-----------|------------|-----------|-------------|
| Shutdown | 0x0006 | 0000 0110 | → Ready to Switch On |
| Switch On | 0x0007 | 0000 0111 | → Switched On |
| Enable Operation | 0x000F | 0000 1111 | → Operation Enabled |
| Disable Voltage | 0x0000 | 0000 0000 | → Switch On Disabled |
| Quick Stop | 0x0002 | 0000 0010 | → Quick Stop Active |
| Fault Reset | 0x0080 | 1000 0000 | 清除故障 |
| New Set Point | 0x0010 | 0001 0000 | 触发新位置（Bit 4） |

### 关键位定义

| **Bit** | **名称** | **说明** |
|--------|---------|---------|
| 0 | Switch On | 开关电机 |
| 1 | Enable Voltage | 使能电压 |
| 2 | Quick Stop | 快速停止 |
| 3 | Enable Operation | 使能操作 |
| 4 | New Set Point | 新设定点（位置模式） |
| 7 | Fault Reset | 故障复位 |

---

## 6️⃣ CiA402 状态字速查

### 状态识别

| **状态** | **状态字** | **掩码** | **期望值** | **描述** |
|---------|-----------|---------|-----------|---------|
| Not Ready to Switch On | 0x0000 | 0x004F | 0x0000 | 未就绪 |
| Switch On Disabled | 0x0040 | 0x004F | 0x0040 | 禁止开启 |
| Ready to Switch On | 0x0021 | 0x006F | 0x0021 | 准备开启 |
| Switched On | 0x0023 | 0x006F | 0x0023 | 已开启 |
| Operation Enabled | 0x0027 | 0x006F | 0x0027 | 操作已启用 ✅ |
| Quick Stop Active | 0x0007 | 0x006F | 0x0007 | 快速停止中 |
| Fault | 0x0008 | 0x0008 | 0x0008 | 故障 ⚠️ |

### 关键位定义

| **Bit** | **名称** | **说明** |
|--------|---------|---------|
| 0 | Ready to Switch On | 准备开启 |
| 1 | Switched On | 已开启 |
| 2 | Operation Enabled | 操作已启用 |
| 3 | Fault | 故障 |
| 5 | Quick Stop | 快速停止 |
| 10 | Target Reached | 目标到达 ✅ |

---

## 7️⃣ CAN 帧格式速查

### COB-ID 映射

| **功能** | **基础 COB-ID** | **计算公式** | **示例 (Node 3)** |
|---------|---------------|-------------|------------------|
| NMT | 0x000 | 固定 | 0x000 |
| SYNC | 0x080 | 固定 | 0x080 |
| EMERGENCY | 0x080 | 0x080 + NodeID | 0x083 |
| TxPDO1 | 0x180 | 0x180 + NodeID | 0x183 |
| RxPDO1 | 0x200 | 0x200 + NodeID | 0x203 |
| RxPDO2 | 0x300 | 0x300 + NodeID | 0x303 |
| TxSDO | 0x580 | 0x580 + NodeID | 0x583 |
| RxSDO | 0x600 | 0x600 + NodeID | 0x603 |
| NMT Error Control | 0x700 | 0x700 + NodeID | 0x703 |

### SDO 帧格式

**SDO 写入（下载）**

```
CAN ID: 0x600 + NodeID
DLC: 8
Data: [CCS, Index_Low, Index_High, Subindex, Data0, Data1, Data2, Data3]
      │    └────┬────┘ └────┬────┘ └──────────┬──────────┘
      │      Index       Subindex            Data
      └─ 命令字节（CCS）：
         0x23 = 4字节加速下载
         0x2B = 2字节加速下载
         0x2F = 1字节加速下载
```

**SDO 读取（上传）**

```
请求：
  CAN ID: 0x600 + NodeID
  Data: [0x40, Index_Low, Index_High, Subindex, 0x00, 0x00, 0x00, 0x00]

响应：
  CAN ID: 0x580 + NodeID
  Data: [0x43, Index_Low, Index_High, Subindex, Data0, Data1, Data2, Data3]
         │
         └─ 0x43 = 4字节加速上传响应
```

### PDO 帧格式

**RxPDO1 (位置控制)**

```
CAN ID: 0x200 + NodeID
DLC: 6
Data: [ControlWord_Low, ControlWord_High, Position0, Position1, Position2, Position3]
      └────────┬────────┘ └─────────────────┬──────────────────┘
          控制字 (16位)              目标位置 (32位)
```

**SYNC 帧**

```
CAN ID: 0x080
DLC: 0
Data: (无)
```

---

## 8️⃣ 角度转换公式速查

### 基本公式

```
命令单位 = 角度 × (target_units_per_rev / gear_ratio) / 360

角度 = 命令单位 × (gear_ratio / target_units_per_rev) × 360
```

### 参数

| **参数** | **默认值** | **说明** |
|---------|-----------|---------|
| `target_units_per_rev` | 10000 | 每圈输出轴的命令单位数 |
| `gear_ratio` (回转/小车) | 10.0 | 减速比 |
| `gear_ratio` (吊钩) | 20.0 | 减速比 |

### 常用换算

| **电机** | **角度输入** | **命令单位输出** | **计算过程** |
|---------|------------|----------------|------------|
| 回转 | 90° | 250 | 90 × (10000/10)/360 = 250 |
| 回转 | 180° | 500 | 180 × (10000/10)/360 = 500 |
| 回转 | 360° | 1000 | 360 × (10000/10)/360 = 1000 |
| 小车 | 90° | 250 | 90 × (10000/10)/360 = 250 |
| 吊钩 | 90° | 125 | 90 × (10000/20)/360 = 125 |

---

## 9️⃣ 命令行速查

### 配置 CAN 接口

```bash
# 加载 CAN 模块
sudo modprobe can
sudo modprobe can_raw

# 配置 can0 (1 Mbps)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 查看接口状态
ip -details link show can0

# 关闭接口
sudo ip link set down can0
```

### ROS 2 命令

```bash
# 发送位置指令
ros2 topic pub /slewing/target_position std_msgs/msg/Float32 "data: 90.0" --once
ros2 topic pub /trolley/target_position std_msgs/msg/Float32 "data: 180.0" --once
ros2 topic pub /hoist/target_position std_msgs/msg/Float32 "data: 45.0" --once

# 监控位置反馈
ros2 topic echo /slewing/crane_position
ros2 topic echo /trolley/crane_position
ros2 topic echo /hoist/crane_position

# 监控电机状态
ros2 topic echo /slewing/crane_status

# 调用服务
ros2 service call /slewing/start_crane std_srvs/srv/Trigger
ros2 service call /slewing/stop_crane std_srvs/srv/Trigger
ros2 service call /slewing/reset_crane std_srvs/srv/Trigger

# 切换模式
ros2 service call /slewing/set_crane_mode std_srvs/srv/SetBool "data: true"  # 位置模式
ros2 service call /slewing/set_crane_mode std_srvs/srv/SetBool "data: false" # 速度模式

# 发送同步轨迹
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

### CAN 工具

```bash
# 监控所有 CAN 帧
candump can0

# 监控特定 CAN ID
candump can0,203:7FF  # RxPDO1 (Node 3)
candump can0,183:7FF  # TxPDO1 (Node 3)
candump can0,603:7FF  # SDO Request (Node 3)
candump can0,583:7FF  # SDO Response (Node 3)
candump can0,080:7FF  # SYNC

# 发送测试帧（手动）
cansend can0 203#0F00FA000000  # RxPDO1: 控制字 0x0F, 位置 250

# CAN 总线统计
canbusload can0@1000000 -r

# 记录 CAN 流量
candump can0 -l
```

---

## 🔟 故障排查速查

### 常见错误代码

| **错误** | **描述** | **解决方案** |
|---------|---------|------------|
| 状态字 0x0008 | 故障状态 | 调用 `/reset_crane` 服务 |
| 状态字 0x0040 | 禁止开启 | 检查硬件使能信号 |
| SDO Abort 0x06040043 | 一般参数错误 | 检查对象字典索引和数据类型 |
| SDO Abort 0x08000020 | 数据不可访问 | 检查电机操作模式 |
| SDO Abort 0x06010000 | 不支持的访问 | 该对象不支持写入 |

### 诊断命令

```bash
# 检查 CAN 接口
ip link show can0

# 检查 ROS 2 节点
ros2 node list
ros2 node info /slewing

# 检查 Topic 连接
ros2 topic list
ros2 topic info /slewing/target_position

# 查看日志
ros2 run crane_master canopen_ros2_node --ros-args --log-level debug

# 检查 CAN 流量
candump can0 -t z -L  # 显示时间戳和长度
```

---

## 1️⃣1️⃣ 性能指标速查

| **指标** | **数值** | **备注** |
|---------|---------|---------|
| **延迟** |  |  |
| ROS 2 Topic 延迟 | < 5 ms | DDS 传输 |
| 应用逻辑处理 | < 1 ms | 角度转换 + 状态机 |
| CAN 帧发送延迟 | < 1 ms | write() 系统调用 |
| 电机响应延迟 | < 20 ms | SYNC 到运动启动 |
| 总端到端延迟 | < 30 ms | 用户输入到电机运动 |
| **带宽** |  |  |
| CAN 波特率 | 1 Mbps | 标准配置 |
| CAN 最大帧率 | ~7500 fps | 理论值（标准帧） |
| PDO 实际帧率 | 100 Hz | SYNC 周期 10ms |
| **分辨率** |  |  |
| 位置分辨率（回转/小车） | 0.036° | 10000 units/rev, ratio 10 |
| 位置分辨率（吊钩） | 0.018° | 10000 units/rev, ratio 20 |
| 速度分辨率 | 0.036 °/s | 与位置分辨率相同 |
| **可靠性** |  |  |
| SDO 超时 | 500 ms | 读取响应最大等待 |
| SDO 重试次数 | 3 次 | 失败后重试 |
| CAN 错误检测 | CRC-15 | 硬件校验 |

---

## 1️⃣2️⃣ 开发调试技巧

### 1. 打印 CAN 帧详情

```bash
# 带时间戳和 ASCII
candump can0 -t a -L

# 解码示例
# 时间戳      ID   DLC  数据
# (000.123456) 203  [6]  0F 00 FA 00 00 00
#              ││││  │   └─────┬──────┘
#              ││││  │      PDO数据
#              ││││  └─ 数据长度
#              │││└─ Node ID 3
#              ││└─ PDO功能码 (0x200)
#              └┴─ CAN ID
```

### 2. 使用 ROS 2 Launch 参数

```bash
# 指定 Node ID
ros2 launch crane_master canopen_ros2.launch.py node_id:=3

# 指定 CAN 接口
ros2 launch crane_master canopen_ros2.launch.py can_interface:=can0

# 自动启动
ros2 launch crane_master canopen_ros2.launch.py auto_start:=true

# 启动所有节点
ros2 launch crane_master canopen_ros2.launch.py node_id:=all
```

### 3. 调试日志级别

```bash
# 设置日志级别为 DEBUG
ros2 run crane_master canopen_ros2_node --ros-args --log-level DEBUG

# 仅显示特定节点日志
ros2 run crane_master canopen_ros2_node --ros-args --log-level INFO \
  --ros-args -r __node:=slewing
```

### 4. 使用 rqt 工具

```bash
# 可视化 Topic 流量
rqt_graph

# 监控 Topic 数值
rqt_plot /slewing/crane_position/data

# 手动发布消息
rqt_publisher
```

---

## 📚 相关文档链接

| **文档** | **路径** |
|---------|---------|
| 完整技术路线 | `/home/labcrane/appdata/ws_tower_crane/CRANE_MASTER_TARGET_POSITION_数据流完整技术路线.md` |
| 可视化流程图 | `/home/labcrane/appdata/ws_tower_crane/CRANE_MASTER_数据流可视化流程图.md` |
| 文档索引 | `/home/labcrane/appdata/ws_tower_crane/src/tower_crane/crane/crane_master/TARGET_POSITION_数据流技术文档索引.md` |
| 架构说明 | `/home/labcrane/appdata/ws_tower_crane/src/tower_crane/crane/crane_master/ARCHITECTURE.txt` |
| README | `/home/labcrane/appdata/ws_tower_crane/src/tower_crane/crane/crane_master/README.md` |

---

**最后更新**：2026-02-03  
**版本**：v1.0  
**维护者**：Crane Master Team

---

💡 **提示**：将本文档打印或保存为 PDF，方便日常开发快速查阅！
