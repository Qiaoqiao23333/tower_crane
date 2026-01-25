#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import re

class CraneJoystickNode(Node):
    def __init__(self):
        super().__init__('crane_joystick_driver')
        
        # === 调试模式 ===
        self.debug_mode = self.declare_parameter('debug', False).value
        self.data_count = 0  # 接收数据计数
        self.publish_count = {'hoist': 0, 'trolley': 0, 'slewing': 0}  # 发布计数

        # === 配置串口 (请根据实际情况修改端口号，通常是 /dev/ttyUSB0) ===
        serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        self.ser = None
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            self.get_logger().info(f'✅ 串口连接成功: {serial_port}')
        except Exception as e:
            self.get_logger().error(f'❌ 无法打开串口 {serial_port}: {e}')
            self.get_logger().error('节点将继续运行，但无法读取串口数据')
            # 不return，让节点继续初始化，但串口为None

        # === 创建发布者 ===
        self.pub_hoist = self.create_publisher(Float32, '/hoist/target_position', 10)
        self.pub_trolley = self.create_publisher(Float32, '/trolley/target_position', 10)
        self.pub_slewing = self.create_publisher(Float32, '/slewing/target_position', 10)

        # === 内部状态记录 ===
        # 假设初始位置 (请根据你的电机实际归零位置调整)
        self.pos_hoist = 0.0
        self.pos_trolley = 0.0
        self.pos_slewing = 0.0

        self.mode = 0 # 0: 控制 Trolley/Slewing, 1: 控制 Hoist
        self.last_sw = 1
        
        # === 控制参数 ===
        # 当前摇杆ADC范围约为 0-1023, 中值约 512
        self.dead_zone = 40      # 死区，防止轻微抖动
        self.center_val = 512    # 摇杆ADC 中值
        self.speed_factor = 5.0  # 灵敏度调整（越大越灵敏，增大以让移动更明显）
        
        # === 位置限制 (防止累积漂移) ===
        self.max_pos = 360.0  # 最大位置限制
        self.min_pos = -360.0  # 最小位置限制

        # 创建定时器读取串口
        self.create_timer(0.05, self.timer_callback) # 20Hz
        
        # 定期输出状态（仅在 debug_mode 下有用）
        self.create_timer(1.0, self.status_callback) # 1Hz

    def map_speed(self, val):
        """将摇杆数值(0-1023)映射为速度增量"""
        diff = val - self.center_val
        if abs(diff) < self.dead_zone:
            return 0.0
        # 归一化：将偏移量(-512到+512)映射到(-1到+1)，然后乘以速度系数
        # 使用 512.0 作为归一化基准（因为范围是0-1023，中值512，最大偏移±512）
        normalized = diff / 512.0
        return normalized * self.speed_factor

    def clamp_position(self, pos):
        """限制位置在合理范围内"""
        return max(self.min_pos, min(self.max_pos, pos))
    
    def status_callback(self):
        """定期报告状态"""
        if self.debug_mode:
            self.get_logger().info(
                f'📊 状态: 接收数据={self.data_count}次, '
                f'发布[Hoist={self.publish_count["hoist"]}, '
                f'Trolley={self.publish_count["trolley"]}, '
                f'Slewing={self.publish_count["slewing"]}], '
                f'位置[H={self.pos_hoist:.2f}°, T={self.pos_trolley:.2f}°, S={self.pos_slewing:.2f}°]'
            )

    def timer_callback(self):
        # 检查串口是否可用
        if self.ser is None or not self.ser.is_open:
            # 每5秒警告一次
            if hasattr(self, '_last_warning_time'):
                import time
                if time.time() - self._last_warning_time > 5.0:
                    self.get_logger().warn('⚠️  串口未连接，无法读取数据')
                    self._last_warning_time = time.time()
            else:
                import time
                self._last_warning_time = time.time()
            return

        try:
            if self.ser.in_waiting > 0:
                # 读取一行数据，使用errors='ignore'防止解码错误
                raw_data = self.ser.readline()
                if len(raw_data) == 0:
                    # 设备报告可读但无数据，可能是断开连接
                    self.get_logger().debug('串口报告可读但无数据，可能已断开连接')
                    return
                line = raw_data.decode('utf-8', errors='ignore').strip()
                
                # 跳过空行
                if not line:
                    return
                
                # 清理数据：移除可能的标签前缀（如 "X:", "Y:", "SW:" 等）
                # 处理格式如 "X:496,Y:509,SW:1" 或 ":495494,Y:495,Y" 或 "495494,495,1"
                cleaned_line = line
                # 移除所有字母数字组合加冒号的标签（如 "X:", "Y:", "SW:" 等）
                cleaned_line = re.sub(r'\w+:', '', cleaned_line)
                # 移除行首的冒号（处理 ":495494" 这种情况）
                cleaned_line = cleaned_line.lstrip(':')
                
                data = cleaned_line.split(',')
                
                if len(data) < 3:
                    self.get_logger().debug(f'数据格式错误，期望至少3个值，得到{len(data)}个: {line}')
                    return

                # 读取原始数据，添加错误处理
                try:
                    # 提取每个字段中的数字部分
                    def extract_number(s):
                        # 移除所有非数字字符（除了负号），只保留数字
                        s = s.strip()
                        # 如果字符串以冒号开头，移除它
                        s = s.lstrip(':')
                        # 提取第一个连续的数字序列
                        match = re.search(r'-?\d+', s)
                        if match:
                            return int(match.group())
                        raise ValueError(f"无法从 '{s}' 中提取数字")
                    
                    raw_x = extract_number(data[0])
                    raw_y = extract_number(data[1])
                    raw_sw = extract_number(data[2])
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f'数据解析失败: {line}, 错误: {e}')
                    return

                # 成功解析一条数据
                self.data_count += 1
                if self.debug_mode:
                    self.get_logger().info(
                        f'📥 接收数据: raw="{line}", cleaned="{cleaned_line}", '
                        f'X={raw_x}, Y={raw_y}, SW={raw_sw}, mode={self.mode}'
                    )

                # === 按钮处理：模式切换 (上升沿触发) ===
                if raw_sw == 0 and self.last_sw == 1:
                    self.mode = 1 - self.mode # 0变1，1变0
                    mode_name = "Hoist (升降)" if self.mode == 1 else "XY (回转/变幅)"
                    self.get_logger().info(f'模式切换为: {mode_name}')
                self.last_sw = raw_sw

                # === 运动计算 ===
                speed_x = self.map_speed(raw_x) # 左右
                speed_y = self.map_speed(raw_y) # 上下

                # 记录旧位置，用于判断是否需要发布
                old_pos_hoist = self.pos_hoist
                old_pos_trolley = self.pos_trolley
                old_pos_slewing = self.pos_slewing

                # 根据模式分配控制权
                if self.mode == 0:
                    # 模式0: X轴控制旋转(Slewing), Y轴控制小车(Trolley)
                    self.pos_slewing = self.clamp_position(self.pos_slewing + speed_x)
                    self.pos_trolley = self.clamp_position(self.pos_trolley + speed_y)
                else:
                    # 模式1: Y轴控制升降(Hoist)
                    self.pos_hoist = self.clamp_position(self.pos_hoist + speed_y)
                    # X轴在升降模式下可以留空，或者做微调

                # === 发布话题 (只在值变化时发布) ===
                if abs(self.pos_hoist - old_pos_hoist) > 0.01:  # 0.01度阈值
                    msg_hoist = Float32()
                    msg_hoist.data = float(self.pos_hoist)
                    self.pub_hoist.publish(msg_hoist)
                    self.publish_count['hoist'] += 1
                    if self.debug_mode:
                        self.get_logger().info(f'📤 发布 Hoist 目标位置: {self.pos_hoist:.2f}°')
                
                if abs(self.pos_trolley - old_pos_trolley) > 0.01:
                    msg_trolley = Float32()
                    msg_trolley.data = float(self.pos_trolley)
                    self.pub_trolley.publish(msg_trolley)
                    self.publish_count['trolley'] += 1
                    if self.debug_mode:
                        self.get_logger().info(f'📤 发布 Trolley 目标位置: {self.pos_trolley:.2f}°')
                
                if abs(self.pos_slewing - old_pos_slewing) > 0.01:
                    msg_slewing = Float32()
                    msg_slewing.data = float(self.pos_slewing)
                    self.pub_slewing.publish(msg_slewing)
                    self.publish_count['slewing'] += 1
                    if self.debug_mode:
                        self.get_logger().info(f'📤 发布 Slewing 目标位置: {self.pos_slewing:.2f}°')

        except serial.SerialException as e:
            self.get_logger().error(f'串口通信错误: {e}')
        except Exception as e:
            self.get_logger().warn(f'未预期的错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CraneJoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在关闭节点...')
    finally:
        # 关闭串口
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
            node.get_logger().info('串口已关闭')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()