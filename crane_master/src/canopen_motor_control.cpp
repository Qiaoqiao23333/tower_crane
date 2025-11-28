#include "crane_master/canopen_ros2_node.hpp"

void CANopenROS2::initialize_node()
{
    // 发送NMT停止命令
    send_nmt_command(NMT_STOP_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 发送NMT重置命令
    send_nmt_command(NMT_RESET_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 先使能电机，再设置操作模式
    // 关闭（Shutdown）
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 准备开启（Switch on）
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 使能操作（Enable operation）
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 读取状态字，确认电机已使能
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "使能后状态字: 0x%04X", status_word);
    
    // 现在尝试设置操作模式
    write_sdo(OD_OPERATION_MODE, 0x00, MODE_PROFILE_POSITION, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 验证操作模式
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "当前操作模式: %d", mode);
    
    // 如果操作模式仍然不是位置模式，尝试使用PDO设置
    if (mode != MODE_PROFILE_POSITION)
    {
        RCLCPP_WARN(this->get_logger(), "使用SDO设置操作模式失败，尝试使用PDO");
        
        // 使用PDO设置操作模式
        struct can_frame frame;
        frame.can_id = COB_RPDO1 + node_id_;
        frame.can_dlc = 3;  // 控制字(2字节) + 操作模式(1字节)
        frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;  // 控制字低字节
        frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;  // 控制字高字节
        frame.data[2] = MODE_PROFILE_POSITION;  // 操作模式
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "发送PDO操作模式失败");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "PDO操作模式已发送");
        }
        
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 再次验证操作模式
        mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "PDO设置后操作模式: %d", mode);
    }
    
    // 设置轮廓速度
    set_profile_velocity(5);
    
    // 设置轮廓加速度
    set_profile_acceleration(5);
    
    // 设置轮廓减速度
    set_profile_deceleration(5);
    
    // 禁用同步生成器
    write_sdo(OD_SYNC_MANAGER, 0x00, 0, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 设置通信周期为1000微秒
    write_sdo(OD_SYNC_MANAGER, 0x00, 1000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "节点初始化完成");
}

void CANopenROS2::configure_pdo()
{
    RCLCPP_INFO(this->get_logger(), "配置PDO映射...");
    
    // 进入预操作状态
    send_nmt_command(NMT_STOP_REMOTE_NODE);
    RCLCPP_INFO(this->get_logger(), "已进入预操作状态");
    
    // 配置TxPDO1
    RCLCPP_INFO(this->get_logger(), "开始配置TxPDO1");
    
    // 1. 禁用TxPDO1
    uint32_t txpdo1_cob_id = COB_TPDO1 + node_id_;
    write_sdo(0x1800, 0x01, txpdo1_cob_id | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. 设置传输类型
    write_sdo(0x1800, 0x02, 0x01, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 3. 清除TxPDO1映射
    write_sdo(0x1A00, 0x00, 0x00, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 4. 设置映射对象：状态字
    write_sdo(0x1A00, 0x01, 0x60410010, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 5. 设置映射对象：实际位置
    write_sdo(0x1A00, 0x02, 0x60640020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 6. 设置TxPDO1映射对象数量为2
    write_sdo(0x1A00, 0x00, 0x02, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 7. 设置传输类型并启用TxPDO1
    write_sdo(0x1800, 0x02, 0xFF, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_sdo(0x1800, 0x01, txpdo1_cob_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 配置RxPDO1
    RCLCPP_INFO(this->get_logger(), "开始配置RxPDO1");
    
    // 1. 禁用RxPDO1
    uint32_t rxpdo1_cob_id = COB_RPDO1 + node_id_;
    write_sdo(0x1400, 0x01, rxpdo1_cob_id | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. 设置传输类型
    write_sdo(0x1400, 0x02, 0x01, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 3. 清除RxPDO1映射
    write_sdo(0x1600, 0x00, 0x00, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 4. 设置映射对象：控制字
    write_sdo(0x1600, 0x01, 0x60400010, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 5. 设置映射对象：目标位置
    write_sdo(0x1600, 0x02, 0x607A0020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 6. 设置RxPDO1映射对象数量为2
    write_sdo(0x1600, 0x00, 0x02, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 7. 设置传输类型并启用RxPDO1
    write_sdo(0x1400, 0x02, 0xFF, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_sdo(0x1400, 0x01, rxpdo1_cob_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "PDO配置完成");
}

void CANopenROS2::start_node()
{
    RCLCPP_INFO(this->get_logger(), "启动节点...");
    
    // 发送NMT启动命令
    send_nmt_command(NMT_START_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 获取实际位置
    int32_t actual_position = read_sdo(OD_ACTUAL_POSITION, 0x00);
    float actual_angle = position_to_angle(actual_position);
    RCLCPP_INFO(this->get_logger(), "实际位置: %.2f°", actual_angle);
    
    // 发送同步帧
    send_sync_frame();
    
    RCLCPP_INFO(this->get_logger(), "节点启动完成");
}

void CANopenROS2::set_immediate_effect(bool immediate)
{
    RCLCPP_INFO(this->get_logger(), "设置%s效果", immediate ? "立即" : "非立即");
    
    // 读取当前控制字
    int32_t controlword = read_sdo(OD_CONTROL_WORD, 0x00);
    
    if (immediate)
    {
        controlword |= (1 << 5);  // 设置位5为1（立即生效）
    }
    else
    {
        controlword &= ~(1 << 5);  // 设置位5为0（非立即生效）
    }
    
    // 写入新的控制字
    write_sdo(OD_CONTROL_WORD, 0x00, controlword, 2);
    
    RCLCPP_INFO(this->get_logger(), "控制字已更新为: 0x%04X", controlword);
}

void CANopenROS2::clear_fault()
{
    RCLCPP_INFO(this->get_logger(), "清除故障...");
    
    // 发送故障复位命令
    set_control_word(CONTROL_FAULT_RESET);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 进入就绪状态
    set_control_word(CONTROL_SHUTDOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "故障已清除");
}

void CANopenROS2::enable_motor()
{
    RCLCPP_INFO(this->get_logger(), "使能电机...");
    
    // 读取当前状态字
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "当前状态字: 0x%04X", status_word);
    
    // 先使用SDO设置控制字
    // 关闭（Shutdown）
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 准备开启（Switch on）
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 使能操作（Enable operation）
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 再次读取状态字，确认电机已使能
    status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "使能后状态字: 0x%04X", status_word);
    
    // 然后使用PDO发送控制字
    set_control_word(CONTROL_SHUTDOWN);  // 关机
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    set_control_word(CONTROL_SWITCH_ON);  // 开启
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    set_control_word(CONTROL_ENABLE_OPERATION);  // 使能操作
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "电机已使能");
}

void CANopenROS2::stop_motor()
{
    RCLCPP_INFO(this->get_logger(), "停止电机...");
    
    // 设置目标速度为0
    set_target_velocity(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 禁用操作
    set_control_word(CONTROL_SWITCH_ON);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 关闭电机
    set_control_word(CONTROL_SHUTDOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 禁用电压
    set_control_word(CONTROL_DISABLE_VOLTAGE);
    
    RCLCPP_INFO(this->get_logger(), "电机已停止");
}

void CANopenROS2::initialize_motor()
{
    // 初始化节点
    initialize_node();
    
    // 配置PDO映射
    configure_pdo();
    
    // 等待一段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 启动节点
    start_node();
    
    // 设置立即生效
    set_immediate_effect(true);
    
    // 清除故障
    clear_fault();
    
    // 使能电机
    enable_motor();
    
    RCLCPP_INFO(this->get_logger(), "电机初始化完成");
}

void CANopenROS2::set_operation_mode(uint8_t mode)
{
    RCLCPP_INFO(this->get_logger(), "开始切换操作模式到: %d", mode);
    
    // 步骤1: 先读取当前状态字和模式
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    int32_t current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "当前状态字: 0x%04X, 当前模式: %d", status_word, current_mode);
    
    // 如果已经是目标模式，直接返回
    if (current_mode == mode)
    {
        RCLCPP_INFO(this->get_logger(), "已经是目标模式: %d", mode);
        return;
    }
    
    // 步骤2: 确保电机处于"Ready to switch on"状态 (0x0021) 或 "Switched on" (0x0023)
    // 这是切换操作模式所需的状态
    if ((status_word & 0x006F) != 0x0021 && (status_word & 0x006F) != 0x0023)
    {
        RCLCPP_INFO(this->get_logger(), "电机不在允许切换模式的状态，正在转换状态...");
        
        // 先禁用操作
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 转换到"Ready to switch on"
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 检查状态
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "状态转换后状态字: 0x%04X", status_word);
    }
    
    // 步骤3: 设置操作模式 (必须在"Ready to switch on"或"Switched on"状态下)
    RCLCPP_INFO(this->get_logger(), "设置操作模式到: %d", mode);
    write_sdo(OD_OPERATION_MODE, 0x00, mode, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // 检查操作模式是否设置成功
    current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "设置后的操作模式: %d", current_mode);
    
    // 如果SDO设置失败，尝试使用PDO
    if (current_mode != mode)
    {
        RCLCPP_WARN(this->get_logger(), "SDO设置模式失败，尝试使用PDO");
        
        // 使用PDO发送操作模式（通过RPDO1，如果映射了操作模式）
        // 注意：这需要RPDO1映射包含操作模式对象
        struct can_frame frame;
        frame.can_id = COB_RPDO1 + node_id_;
        frame.can_dlc = 3;  // 控制字(2字节) + 操作模式(1字节)
        frame.data[0] = CONTROL_SWITCH_ON & 0xFF;  // 控制字低字节
        frame.data[1] = (CONTROL_SWITCH_ON >> 8) & 0xFF;  // 控制字高字节
        frame.data[2] = mode;  // 操作模式
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "发送PDO操作模式失败");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "PDO操作模式已发送: %d", mode);
        }
        
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // 再次检查操作模式
        current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "PDO设置后的操作模式: %d", current_mode);
    }
    
    // 步骤4: 如果模式设置成功，重新使能电机到操作状态
    if (current_mode == mode)
    {
        RCLCPP_INFO(this->get_logger(), "操作模式设置成功，重新使能电机");
        
        // 状态机转换: Shutdown -> Switch on -> Enable operation
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 最终确认
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "最终状态字: 0x%04X", status_word);
        RCLCPP_INFO(this->get_logger(), "操作模式切换成功: %d", mode);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "操作模式切换失败，当前模式: %d, 期望模式: %d", 
                    current_mode, mode);
        RCLCPP_ERROR(this->get_logger(), "请检查电机是否支持模式 %d，或查看电机文档", mode);
    }
}

void CANopenROS2::set_profile_velocity(float velocity_deg_per_sec)
{
    int32_t velocity_pulse = velocity_to_pulse(velocity_deg_per_sec);
    write_sdo(OD_PROFILE_VELOCITY, 0x00, velocity_pulse, 4);
    RCLCPP_INFO(this->get_logger(), "轮廓速度已设置: %.2f°/s", velocity_deg_per_sec);
}

void CANopenROS2::set_profile_acceleration(float acceleration_deg_per_sec2)
{
    int32_t acceleration_pulse = acceleration_to_pulse(acceleration_deg_per_sec2);
    write_sdo(OD_PROFILE_ACCELERATION, 0x00, acceleration_pulse, 4);
    RCLCPP_INFO(this->get_logger(), "轮廓加速度已设置: %.2f°/s²", acceleration_deg_per_sec2);
}

void CANopenROS2::set_profile_deceleration(float deceleration_deg_per_sec2)
{
    int32_t deceleration_pulse = acceleration_to_pulse(deceleration_deg_per_sec2);
    write_sdo(OD_PROFILE_DECELERATION, 0x00, deceleration_pulse, 4);
    RCLCPP_INFO(this->get_logger(), "轮廓减速度已设置: %.2f°/s²", deceleration_deg_per_sec2);
}

void CANopenROS2::set_profile_parameters(float velocity_deg_per_sec, float acceleration_deg_per_sec2, float deceleration_deg_per_sec2)
{
    set_profile_velocity(velocity_deg_per_sec);
    set_profile_acceleration(acceleration_deg_per_sec2);
    set_profile_deceleration(deceleration_deg_per_sec2);
    RCLCPP_INFO(this->get_logger(), "轮廓参数设置完成 - 速度: %.2f°/s, 加速度: %.2f°/s², 减速度: %.2f°/s²", 
               velocity_deg_per_sec, acceleration_deg_per_sec2, deceleration_deg_per_sec2);
}

void CANopenROS2::set_control_word(uint16_t control_word)
{
    // 使用PDO发送控制字
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 2;
    frame.data[0] = control_word & 0xFF;  // 控制字低字节
    frame.data[1] = (control_word >> 8) & 0xFF;  // 控制字高字节
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "发送控制字失败");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "控制字已发送: 0x%04X", control_word);
    }
}

void CANopenROS2::set_target_velocity(int32_t velocity_pulse_per_sec)
{
    // DSY-C.EDS specifies target velocity at 0x60FF
    write_sdo(OD_TARGET_VELOCITY, 0x00, velocity_pulse_per_sec, 4);
    RCLCPP_INFO(this->get_logger(), "目标速度已设置(脉冲): %d", velocity_pulse_per_sec);
}

void CANopenROS2::go_to_position(float angle)
{
    RCLCPP_INFO(this->get_logger(), "移动到位置: %.2f°", angle);
    
    int32_t position = angle_to_position(angle);
    RCLCPP_INFO(this->get_logger(), "目标位置脉冲值: %d", position);
    
    // 先使用SDO设置目标位置
    write_sdo(OD_TARGET_POSITION, 0x00, position, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 使用SDO设置控制字，触发位置命令
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 重置控制字
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 然后使用PDO发送目标位置
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 6;  // 控制字(2字节) + 目标位置(4字节)
    frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;  // 控制字低字节
    frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;  // 控制字高字节
    frame.data[2] = position & 0xFF;  // 目标位置低字节
    frame.data[3] = (position >> 8) & 0xFF;
    frame.data[4] = (position >> 16) & 0xFF;
    frame.data[5] = (position >> 24) & 0xFF;  // 目标位置高字节
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "发送目标位置失败");
        return;
    }
    
    send_sync_frame();
    
    // 先重置命令触发位（位4）
    set_control_word(CONTROL_ENABLE_OPERATION);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 设置命令触发位，创建上升沿
    set_control_word(CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT);
    send_sync_frame();
    
    RCLCPP_INFO(this->get_logger(), "位置命令已发送");
    
    // 如果目标位置就是当前位置，不需要等待
    int32_t current_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
    int32_t position_diff = (position > current_pos) ? (position - current_pos) : (current_pos - position);
    if (position_diff < 100)  // 如果位置差小于100个脉冲（约0.27度），认为已到达
    {
        RCLCPP_INFO(this->get_logger(), "目标位置与当前位置接近，无需移动");
        return;
    }
    
    // 监控目标位置是否到达
    int retry = 0;
    const int max_retries = 50;  // 增加到50次，总共10秒
    while (retry < max_retries)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 读取状态字
        int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
        
        // 读取实际位置
        int32_t actual_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
        int32_t current_diff = (position > actual_pos) ? (position - actual_pos) : (actual_pos - position);
        
        // 检查目标到达位（位10）或位置误差足够小
        if ((status_word & 0x0400) || current_diff < 100)
        {
            RCLCPP_INFO(this->get_logger(), "目标位置已到达 (状态字: 0x%04X, 位置差: %d)", status_word, current_diff);
            break;
        }
        
        // 每5次重试输出一次进度
        if (retry % 5 == 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "等待目标到达... (重试 %d/%d, 位置差: %d)", retry, max_retries, current_diff);
        }
        
        retry++;
    }
    
    if (retry >= max_retries)
    {
        int32_t final_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
        int32_t final_diff = (position > final_pos) ? (position - final_pos) : (final_pos - position);
        RCLCPP_WARN(this->get_logger(), "等待目标位置到达超时 (最终位置差: %d 脉冲, 约 %.2f°)", 
                   final_diff, position_to_angle(final_diff));
    }
}

void CANopenROS2::set_velocity(float velocity_deg_per_sec)
{
    RCLCPP_INFO(this->get_logger(), "设置速度: %.2f°/s", velocity_deg_per_sec);
    
    // 读取当前操作模式
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    
    if (mode != MODE_PROFILE_VELOCITY && mode != MODE_VELOCITY)
    {
        RCLCPP_WARN(this->get_logger(), "当前不是速度模式，无法设置速度。当前模式: %d", mode);
        return;
    }
    
    // 转换为电机内部单位
    int32_t velocity_pulse = velocity_to_pulse(velocity_deg_per_sec);
    
    // 设置目标速度
    write_sdo(0x60FF, 0x00, velocity_pulse, 4);  // 0x60FF是目标速度对象
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 使能操作
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    
    RCLCPP_INFO(this->get_logger(), "速度已设置: %.2f°/s (脉冲值: %d)", velocity_deg_per_sec, velocity_pulse);
}

void CANopenROS2::set_velocity_pdo(float velocity_deg_per_sec)
{
    RCLCPP_INFO(this->get_logger(), "使用PDO设置速度: %.2f°/s", velocity_deg_per_sec);
    
    // 读取当前操作模式
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "当前操作模式: %d", mode);
    
    // 如果当前不是速度模式，需要先切换到速度模式
    if (mode != MODE_VELOCITY && mode != MODE_PROFILE_VELOCITY)
    {
        RCLCPP_INFO(this->get_logger(), "当前模式不是速度模式，正在切换到速度模式...");
        
        // 尝试切换到速度模式 (MODE_VELOCITY = 2)
        set_operation_mode(MODE_VELOCITY);
        
        // 再次检查模式
        mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "切换后操作模式: %d", mode);
        
        if (mode != MODE_VELOCITY && mode != MODE_PROFILE_VELOCITY)
        {
            RCLCPP_ERROR(this->get_logger(), "无法切换到速度模式，当前模式: %d。速度设置可能失败。", mode);
            // 继续尝试设置速度，但可能会失败
        }
    }
    
    // 设置轮廓速度参数（用于速度模式）
    set_profile_velocity(velocity_deg_per_sec);
    
    // 转换为电机内部单位
    int32_t velocity_pulse = velocity_to_pulse(velocity_deg_per_sec);
    
    // 使用SDO设置目标速度
    write_sdo(OD_TARGET_VELOCITY, 0x00, velocity_pulse, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 确保电机处于操作使能状态
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    if ((status_word & 0x006F) != 0x0027)  // 如果不是"操作已启用"状态
    {
        RCLCPP_INFO(this->get_logger(), "电机未处于操作状态，正在使能...");
        // 状态机转换
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    else
    {
        // 如果已经在操作状态，只需发送使能操作命令
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO(this->get_logger(), "速度命令已发送: %.2f°/s (脉冲值: %d)", velocity_deg_per_sec, velocity_pulse);
}

