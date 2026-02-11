# 摇杆控制调试指南

当摇杆移动但电机不移动时，按以下步骤排查：

## 快速检查清单

### 步骤 1: 检查节点是否运行

```bash
# 检查所有节点
ros2 node list

# 应该看到：
# /crane_joystick_driver
# /hoist/canopen_ros2_node
# /trolley/canopen_ros2_node
# /slewing/canopen_ros2_node
```

### 步骤 2: 检查串口连接

```bash
# 检查串口设备是否存在
ls -l /dev/ttyUSB*

# 检查权限（应该是 dialout 组）
ls -l /dev/ttyUSB0

# 如果权限不对，添加用户到dialout组：
sudo usermod -a -G dialout $USER
# 然后重新登录或执行：
newgrp dialout
```

### 步骤 3: 启用调试模式运行摇杆节点

```bash
# 使用调试模式运行（会显示详细日志）
ros2 run crane_master joystick_bridge.py --ros-args -p debug:=true
```

你应该看到：
- ✅ 串口连接成功
- 📥 接收数据: X=..., Y=..., SW=...
- 📤 发布目标位置消息

### 步骤 4: 检查话题数据流

**终端1: 监听目标位置话题**
```bash
# 监听回转电机
ros2 topic echo /slewing/target_position

# 或监听所有目标位置
ros2 topic echo /hoist/target_position
ros2 topic echo /trolley/target_position
```

**终端2: 运行摇杆节点**
```bash
ros2 run crane_master joystick_bridge.py --ros-args -p debug:=true
```

**移动摇杆**，你应该在终端1看到数据更新。

### 步骤 5: 检查CANopen节点是否接收数据

```bash
# 检查节点信息
ros2 node info /slewing/canopen_ros2_node

# 查看节点日志（在运行launch文件的终端中）
# 应该看到类似：
# "收到目标位置: XX.XX°"
```

### 步骤 6: 检查电机状态

```bash
# 检查电机是否已初始化
ros2 service call /slewing/init std_srvs/srv/Trigger

# 检查电机状态
ros2 topic echo /slewing/status_word

# 检查当前位置
ros2 topic echo /slewing/position
```

### 步骤 7: 手动测试电机控制

如果摇杆不工作，先手动测试电机是否能正常控制：

```bash
# 手动发送位置命令
ros2 topic pub /slewing/target_position std_msgs/msg/Float32 "data: 45.0" --once

# 如果电机移动，说明CANopen节点正常，问题在摇杆节点
# 如果电机不移动，检查CANopen节点和电机连接
```

## 常见问题

### 问题1: 串口连接失败

**症状**: 看到 "❌ 无法打开串口"

**解决方案**:
1. 检查串口设备路径是否正确（默认是 `/dev/ttyUSB0`）
2. 如果不同，运行时指定：
   ```bash
   ros2 run crane_master joystick_bridge.py --ros-args -p serial_port:=/dev/ttyUSB1
   ```
3. 检查权限：`ls -l /dev/ttyUSB0`
4. 检查是否被其他程序占用：`lsof /dev/ttyUSB0`

### 问题2: 接收不到串口数据

**症状**: 节点运行但没有 "📥 接收数据" 日志

**检查**:
1. STM32是否正在发送数据？
2. 数据格式是否正确？应该是：`x,y,sw\n`（三个整数，逗号分隔，换行结尾）
3. 波特率是否匹配？（默认115200）
4. 可以用 `minicom` 或 `screen` 直接查看串口：
   ```bash
   sudo minicom -D /dev/ttyUSB0 -b 115200
   ```

### 问题3: 数据接收但电机不移动

**症状**: 看到 "📤 发布" 日志，但电机不动

**检查**:
1. CANopen节点是否运行？`ros2 node list`
2. 电机是否已初始化？`ros2 service call /slewing/init std_srvs/srv/Trigger`
3. CAN总线是否正常？`ip -details link show can0`
4. 查看CANopen节点日志，看是否收到位置命令

### 问题4: 位置值不变化

**症状**: 摇杆移动但位置值不变

**可能原因**:
1. 死区太大（默认300），摇杆移动幅度太小
2. 速度系数太小（默认0.5），调整参数：
   ```python
   # 在joystick_bridge.py中修改
   self.dead_zone = 200  # 减小死区
   self.speed_factor = 1.0  # 增加灵敏度
   ```

## 使用调试脚本

运行自动调试脚本：

```bash
# 确保脚本有执行权限
chmod +x src/tower_crane/crane/crane_master/scripts/debug_joystick.sh

# 运行调试脚本
./src/tower_crane/crane/crane_master/scripts/debug_joystick.sh
```

## 完整测试流程

1. **启动CANopen节点**:
   ```bash
   ros2 launch crane_master canopen_ros2.launch.py can_interface:=can0 node_id:=all
   ```

2. **在另一个终端运行摇杆节点（调试模式）**:
   ```bash
   source install/setup.bash
   ros2 run crane_master joystick_bridge.py --ros-args -p debug:=true
   ```

3. **在第三个终端监听话题**:
   ```bash
   ros2 topic echo /slewing/target_position
   ```

4. **移动摇杆**，观察：
   - 摇杆节点终端：应该看到接收和发布日志
   - 话题监听终端：应该看到位置值变化
   - CANopen节点终端：应该看到"收到目标位置"日志

5. **检查电机**：电机应该开始移动

## 参数说明

摇杆节点支持以下参数：

- `serial_port` (默认: `/dev/ttyUSB0`): 串口设备路径
- `debug` (默认: `false`): 是否启用调试模式

使用示例：
```bash
ros2 run crane_master joystick_bridge.py --ros-args \
  -p serial_port:=/dev/ttyUSB1 \
  -p debug:=true
```
