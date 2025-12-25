# Controller Manager 启动问题修复说明

## 问题诊断

你的系统在启动时卡在了这里：
```
[WARN] [hook_joint]: Wait for device to boot...
[INFO] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
```

### 根本原因

1. **CANopen 设备状态问题**
   - 设备在 CAN 总线上处于 **Pre-operational** 状态（心跳值 `0x7F`）
   - ros2_canopen 等待设备启动到 **Operational** 状态（心跳值 `0x05`）
   - Master 节点没有自动发送 NMT Start 命令

2. **配置文件问题**
   - `boot_timeout_ms` 设置过长（60秒）
   - SDO 初始化命令包含了可能失败的 Fault Reset
   - 需要重新编译才能使配置生效

3. **启动顺序问题**
   - Controller Manager 需要等待硬件初始化完成
   - Spawner 需要等待 Controller Manager 服务可用

## 已做的修复

### 1. 优化 bus.yml 配置

**修改了以下参数：**
- ✅ 减少 `boot_timeout_ms`: 从 60000ms 降至 5000ms
- ✅ 减少 `sdo_timeout_ms`: 从 10000ms 降至 5000ms  
- ✅ 移除 SDO 初始化中的 Fault Reset (0x6040=128)，避免启动时冲突
- ✅ 优化 Master 启动超时: 从 10000ms 降至 2000ms

### 2. 文件位置
```
修改的文件：
/home/qiaoqiaochen/appdata/canros/src/crane/tower_crane/config/robot_control/bus.yml
```

## 如何使用

### 方法 1: 使用自动化脚本（推荐）

```bash
cd /home/qiaoqiaochen/appdata/canros/src
./rebuild_and_launch.sh
```

这个脚本会自动：
1. 清理旧的构建文件
2. 重新编译 tower_crane 包（生成新的 master.dcf）
3. 加载环境变量
4. 检查 CAN 接口
5. 启动硬件系统

### 方法 2: 手动步骤

```bash
# 1. 切换到工作空间
cd /home/qiaoqiaochen/appdata/canros

# 2. 重新编译（重要！生成新的 master.dcf）
colcon build --packages-select tower_crane --cmake-clean-cache

# 3. 加载环境
source install/setup.bash

# 4. 启动系统
ros2 launch tower_crane hardware_bringup_real.launch.py
```

### 方法 3: 使用预启动选项（如果方法1/2仍然卡住）

如果设备仍然无法正常启动，使用预启动驱动器选项：

```bash
ros2 launch tower_crane hardware_bringup_real.launch.py pre_enable_drives:=true
```

这会在启动前通过 `cansend` 手动发送 CiA402 使能序列。

## 验证步骤

### 1. 检查 CAN 设备状态

```bash
# 查看心跳消息（应该看到 05 而不是 7F）
timeout 2s candump can0 | grep -E "70[123]"
```

正常输出应该是：
```
can0  701   [1]  05    # 节点 1 (hook_joint) 在 Operational
can0  702   [1]  05    # 节点 2 (trolley_joint) 在 Operational  
can0  703   [1]  05    # 节点 3 (slewing_joint) 在 Operational
```

### 2. 检查 Controller Manager 服务

```bash
# 等待系统启动后执行
ros2 service list | grep controller_manager
```

应该看到：
```
/controller_manager/configure_controller
/controller_manager/list_controllers
/controller_manager/load_controller
...
```

### 3. 检查控制器状态

```bash
ros2 control list_controllers
```

应该看到：
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_position_controller[forward_command_controller/ForwardCommandController] active
```

## 技术细节

### CANopen NMT 状态机

```
设备启动流程：
0x00 (Bootup) → 0x7F (Pre-operational) → 0x05 (Operational)
                                          ↑
                              需要 NMT Start 命令
```

### master.dcf 生成机制

```cmake
# CMakeLists.txt (第 22 行)
generate_dcf(robot_control)  # 从 bus.yml 生成 master.dcf
```

这就是为什么修改 `bus.yml` 后必须重新编译！

### Launch 文件时序

```
1. robot_state_publisher (立即启动)
2. ros2_control_node (立即启动)
   ├─ 加载 CanopenSystem 硬件接口
   ├─ 初始化 Master (启动 NMT)
   ├─ 初始化 Devices (等待 boot_timeout)
   └─ 配置 Controllers
3. spawner (5秒后启动 joint_state_broadcaster)
4. spawner (6秒后启动 forward_position_controller)
```

## 常见问题排查

### Q1: 系统仍然卡在 "Wait for device to boot"

**解决方案：**
```bash
# 手动发送 NMT Start 命令
cansend can0 000#0101  # 启动节点 1
cansend can0 000#0102  # 启动节点 2
cansend can0 000#0103  # 启动节点 3
```

或使用 `pre_enable_drives:=true` 选项。

### Q2: Controller Manager 服务不可用

**检查硬件初始化：**
```bash
# 查看 ros2_control_node 日志
ros2 run controller_manager ros2_control_node --ros-args --log-level debug
```

### Q3: 编译后仍然使用旧配置

**确保完全清理：**
```bash
rm -rf build/tower_crane install/tower_crane
colcon build --packages-select tower_crane --cmake-clean-cache
```

### Q4: CAN 接口错误

```bash
# 重置 CAN 接口
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

# 检查接口状态
ip link show can0
```

## 下一步

系统成功启动后，你可以：

1. **发送位置命令：**
   ```bash
   ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
     "data: [0.0, 0.5, 0.0]" --once
   ```

2. **监控关节状态：**
   ```bash
   ros2 topic echo /joint_states
   ```

3. **使用 MoveIt 规划：**
   ```bash
   ros2 launch tower_crane_moveit_config demo.launch.py use_sim_time:=false
   ```

## 总结

- ✅ 修复了 bus.yml 配置，减少超时并移除问题 SDO 命令
- ✅ 创建了自动化启动脚本 `rebuild_and_launch.sh`
- ✅ 提供了多种启动方法和故障排查步骤
- ✅ 解释了 master.dcf 生成机制和为什么需要重新编译

**最重要的是：修改 bus.yml 后必须重新编译！**

