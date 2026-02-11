# 电机不动问题 - 完整诊断报告

## 问题描述
运行 `hardware_bringup_real.launch.py` 后，系统启动成功，控制器接收轨迹命令，但**电机根本不移动**。

## 诊断过程

### 1. ✅ 电机通信正常
```bash
cansend can0 601#4041600000000000
candump can0,581:7FF -n 1
# 响应: 581 [8] 4B 41 60 00 37 02 00 00
# 状态字 0x0237 = Operation Enabled ✅
```

### 2. ✅ 控制器发送命令
```bash
ros2 action send_goal /forward_position_controller/follow_joint_trajectory ...
# desired position: 0.0 → 0.5 弧度 ✅
# actual position: 始终 0.0 ❌
# 系统报告: "Goal reached, success!" (误报)
```

### 3. ❌ 缺少 RPDO 消息
CAN 总线流量分析：
- **TPDO (0x281, 0x282, 0x283)**: ✅ 存在 - 电机发送状态
- **RPDO (0x201, 0x202, 0x203)**: ❌ **缺失** - 控制器未发送位置命令！

### 4. 🔧 发现的配置问题

#### 问题 A: URDF 插件名称错误
```xml
<!-- 错误 -->
<plugin>canopen_ros2_control/RobotSystem</plugin>

<!-- 正确 -->
<plugin>canopen_ros2_control/CanopenSystem</plugin>
```
**已修复**: `/home/labcrane/appdata/ws_tower_crane/src/tower_crane/crane/tower_crane/urdf/Tower_crane_canopen.urdf.xacro`

#### 问题 B: .bin 文件为空
所有 `.bin` 文件都是 0 字节，因为 `dcfgen` 工具未正确运行。

**根本原因**: CMakeLists.txt 调用 `generate_dcf(robot_control)`，但 `dcfgen` 命令在 PATH 中不可用。

**解决方案**:
```bash
cd /home/labcrane/appdata/ws_tower_crane/src/tower_crane/crane/tower_crane/config/robot_control
export PYTHONPATH=/home/labcrane/appdata/ws_canopen/build/lely_core_libraries/upstream/python/dcf-tools:$PYTHONPATH
python3 -c "from dcfgen.cli import main; import sys; sys.argv = ['dcfgen', '-v', '-d', '.', '-rS', 'bus.yml']; main()"
```

生成的文件：
- `hook_joint.bin` (260 bytes) ✅
- `trolley_joint.bin` (260 bytes) ✅
- `slewing_joint.bin` (260 bytes) ✅
- `master.dcf` (32 KB) ✅

#### 问题 C: EDS 文件缺失
`DSY-C.EDS` 文件未安装到 install 目录。

**已修复**: 创建符号链接
```bash
ln -sf /home/labcrane/appdata/ws_tower_crane/src/tower_crane/crane/tower_crane/config/robot_control/DSY-C.EDS \
       /home/labcrane/appdata/ws_tower_crane/install/tower_crane/share/tower_crane/config/robot_control/DSY-C.EDS
```

### 5. 🚨 当前问题: Boot Timeout

修复上述问题后，系统现在尝试通过 CAN 下载配置到电机，但失败：

```
[ERROR] [hook_joint]: Boot attempt 1 failed: Boot Timeout: The boot configure process timeout!
[ERROR] [hook_joint]: Boot attempt 2 failed: Boot Issue: Configuration download failed.
[ERROR] [hook_joint]: Boot attempt 3 failed: Boot Issue: Configuration download failed.
```

CAN 流量显示心跳消息 (0x701, 0x702, 0x703 = 0x7F = Pre-operational)，但配置下载失败。

## 根本原因分析

**核心问题**: `CanopenSystem` 使用的是 **SDO 轮询模式** (polling: true)，而不是 **PDO 模式**。

在 SDO 轮询模式下：
1. 系统通过 SDO 读取状态
2. 系统通过 SDO 写入命令
3. **但实际的位置命令没有通过 RPDO 发送**

这就是为什么：
- 电机状态可以读取 ✅
- 控制器认为命令已发送 ✅  
- 但电机实际上没有收到位置命令 ❌

## 解决方案选项

### 选项 1: 使用 PDO 模式 (推荐)
修改 `bus.yml`:
```yaml
polling: false  # 使用 PDO 而不是 SDO 轮询
```

**但是**: 这需要正确配置 RPDO/TPDO 映射，并且 `CanopenSystem` 必须正确导出接口。

### 选项 2: 跳过配置下载
某些电机驱动器已经预配置，不需要/不支持配置下载。

修改 `bus.yml`:
```yaml
boot_timeout_ms: 0  # 跳过启动配置
```

### 选项 3: 增加超时时间
如果配置下载只是慢：
```yaml
boot_timeout_ms: 20000  # 20 秒
sdo_timeout_ms: 10000   # 10 秒
```

### 选项 4: 使用原始工作方式
您的原始日志显示系统能够启动（虽然电机不动）。关键区别可能是：
- 使用了不同的硬件接口插件
- 或者使用了不同的 launch 配置

## 下一步行动

1. **立即测试**: 尝试将 `boot_timeout_ms` 设置为 0 或非常大的值
2. **验证 PDO**: 确认电机驱动器是否支持 PDO 模式
3. **检查原始配置**: 对比您之前工作的配置（如果有）

## 相关文件

- URDF: `crane/tower_crane/urdf/Tower_crane_canopen.urdf.xacro`
- 配置: `crane/tower_crane/config/robot_control/bus.yml`
- 启动: `crane/tower_crane/launch/hardware_bringup_real.launch.py`
- CMake: `crane/tower_crane/CMakeLists.txt`

## 版本信息

- `canopen_ros2_control`: 0.2.13
- `lely_core_libraries`: dcf-tools 2.4.0
- ROS 2: Humble


