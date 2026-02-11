# 电机 SDO 通信故障排查指南

## 错误描述

```
[ros2_control_node-2] [ERROR] [slewing_joint]: AsyncDownload:03:6040:00: 
Resource not available: SDO connection (060A0023): 
Resource not available: SDO connection
```

## 问题分析

**根本原因**: CANopen 节点(电机驱动器)没有响应 SDO(Service Data Object)通信请求。

### 技术细节
- **出错节点**: slewing_joint (节点 ID 3)
- **访问对象**: 0x6040 (Controlword 控制字)
- **错误代码**: 060A0023 (Resource not available)
- **错误类型**: lely::canopen::SdoError

这个错误发生在 ros2_control_node 启动时,系统尝试通过 SDO 向电机下载配置参数时失败。

## 故障排查步骤

### 步骤 1: 运行诊断脚本

首先运行我创建的诊断脚本:

```bash
cd /home/labcrane/appdata/ws_tower_crane
chmod +x fix_motor_communication.sh
./fix_motor_communication.sh
```

这个脚本会:
1. ✅ 检查 CAN 接口状态
2. ✅ 验证 CAN 总线流量
3. 🔄 发送 NMT 复位和启动命令
4. 🔍 测试每个电机节点的 SDO 响应
5. 🔧 尝试发送故障复位命令

### 步骤 2: 检查硬件连接

如果脚本显示"无 SDO 响应",请检查:

#### 2.1 电机驱动器电源
```bash
# 检查电机驱动器是否上电
# - 查看驱动器 LED 指示灯是否亮起
# - 测量供电电压 (通常 24V 或 48V)
```

#### 2.2 CAN 总线连接
- **CAN-H 和 CAN-L 线缆**: 确保连接牢固,没有断线
- **终端电阻**: CAN 总线两端应各有一个 120Ω 终端电阻
- **屏蔽和接地**: 确保 CAN 线缆屏蔽层正确接地

```bash
# 测量 CAN_H 和 CAN_L 之间的电阻
# 应该显示约 60Ω (两个 120Ω 终端电阻并联)
```

#### 2.3 节点 ID 配置
检查电机驱动器的 CANopen 节点 ID 设置:
- hook_joint: 节点 ID = 1
- trolley_joint: 节点 ID = 2
- slewing_joint: 节点 ID = 3

### 步骤 3: 使用备用配置(跳过启动配置)

如果电机已经预配置好,可以跳过启动时的 SDO 配置下载:

```bash
# 备份原配置
cd /home/labcrane/appdata/ws_tower_crane/src/tower_crane/crane/tower_crane/config/robot_control
cp bus.yml bus.yml.backup

# 使用无启动配置版本
cp bus_no_boot.yml bus.yml

# 重新编译
cd /home/labcrane/appdata/ws_tower_crane
colcon build --packages-select tower_crane

# 重新启动
source install/setup.bash
ros2 launch tower_crane hardware_bringup_real.launch.py
```

### 步骤 4: 增加超时时间

如果电机响应很慢,可以尝试增加超时:

编辑 `bus.yml`:
```yaml
defaults:
  sdo_timeout_ms: 20000      # 从 5000 增加到 20000 (20秒)
  boot_timeout_ms: 30000     # 从 15000 增加到 30000 (30秒)
```

### 步骤 5: 手动电机复位

某些电机驱动器需要手动复位:

#### 5.1 通过 CAN 发送复位命令
```bash
# 节点 3 (slewing_joint) 的故障复位
cansend can0 603#2B40600080000000
sleep 0.5

# 关机命令 (Shutdown)
cansend can0 603#2B40600006000000
sleep 0.5

# 启用命令 (Switch On)
cansend can0 603#2B40600007000000
sleep 0.5

# 使能操作 (Enable Operation)
cansend can0 603#2B4060000F000000
```

#### 5.2 物理复位
如果软件复位无效:
1. 关闭电机驱动器电源
2. 等待 10 秒
3. 重新上电
4. 等待驱动器初始化完成(通常 5-10 秒)
5. 再次尝试启动 ROS2 系统

### 步骤 6: 检查 CAN 波特率

确认 CAN 总线波特率匹配:

```bash
# 检查当前 CAN 接口配置
ip -details link show can0

# 应该显示: bitrate 500000
# 如果不是,重新配置:
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

同时确认电机驱动器的 CANopen 波特率也设置为 500 kbit/s。

## 解决方案总结

### 方案 A: 跳过启动配置 (推荐首选)
**适用场景**: 电机驱动器已经预配置好参数

**步骤**:
1. 使用 `bus_no_boot.yml` 替换 `bus.yml`
2. 重新编译并启动

**优点**: 避免 SDO 配置下载,启动更快更可靠
**缺点**: 电机参数必须预先在驱动器上配置

### 方案 B: 增加超时并修复通信
**适用场景**: 电机能响应但很慢,或偶尔通信失败

**步骤**:
1. 运行 `fix_motor_communication.sh` 复位电机
2. 修改 `bus.yml` 增加超时时间
3. 重新编译并启动

### 方案 C: 物理检查和复位
**适用场景**: 完全无法通信

**步骤**:
1. 检查所有硬件连接
2. 断电重启电机驱动器
3. 验证节点 ID 和波特率配置
4. 再次尝试方案 A 或 B

## 常见错误和解决方法

### 错误 1: "No response" (无响应)
**原因**: 电机未上电或 CAN 连接断开
**解决**: 检查电源和 CAN 线缆

### 错误 2: "Boot Timeout"
**原因**: SDO 配置下载超时
**解决**: 使用方案 A (跳过启动配置)

### 错误 3: "Resource not available"
**原因**: 电机不支持 SDO 或处于错误状态
**解决**: 
- 发送 NMT Start 命令: `cansend can0 000#0100`
- 或使用方案 A (跳过启动配置)

### 错误 4: CAN 接口状态为 DOWN
**原因**: CAN 接口未启动或错误过多导致 bus-off
**解决**:
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
```

## 监控和日志

### 实时监控 CAN 流量
```bash
# 完整流量
candump can0

# 仅监控节点 3
candump can0,583:7FF,603:7FF,283:7FF

# 记录到文件
candump -l can0
```

### 查看 ROS2 控制器状态
```bash
# 检查控制器
ros2 control list_controllers

# 检查硬件接口
ros2 control list_hardware_interfaces

# 检查关节状态
ros2 topic echo /joint_states
```

## 参考资料

- **CANopen 错误代码**: 060A0023 = SDO Timeout / Resource not available
- **CiA 402 状态机**: CANopen 电机驱动器标准状态转换
- **配置文件位置**: 
  - `src/tower_crane/crane/tower_crane/config/robot_control/bus.yml`
  - `src/tower_crane/crane/tower_crane/launch/hardware_bringup_real.launch.py`

## 联系支持

如果上述步骤都无法解决问题,请收集以下信息:

1. `fix_motor_communication.sh` 的完整输出
2. CAN 流量日志: `candump -l can0` (运行 30 秒)
3. 电机驱动器型号和固件版本
4. 电机驱动器显示屏上的错误代码(如果有)
5. ROS2 启动日志完整输出

---

**最后更新**: 2026-01-13
**版本**: 1.0
