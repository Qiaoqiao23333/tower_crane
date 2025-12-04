#include "crane_master/canopen_ros2_node.hpp"
#include <cmath>

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
    write_sdo(OD_OPERATION_MODE, 0x00, MODE_PROFILE_POSITION, 1); //default is position mode
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
    set_profile_velocity(30);  // 默认速度：30°/s
    
    // 设置轮廓加速度
    set_profile_acceleration(30);  // 默认加速度：30°/s²
    
    // 设置轮廓减速度
    set_profile_deceleration(30);
    
    // 禁用同步生成器
    write_sdo(OD_CYCLE_PERIOD, 0x00, 0, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 设置通信周期为1000微秒
    write_sdo(OD_CYCLE_PERIOD, 0x00, 1000, 4);
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
    
    // 配置RxPDO2 for Velocity Control
    RCLCPP_INFO(this->get_logger(), "配置RxPDO2 for 速度控制");
    
    // 1. Disable RxPDO2 (COB-ID 0x300 + NodeID)
    uint32_t rxpdo2_cob_id = 0x300 + node_id_;
    write_sdo(0x1401, 0x01, rxpdo2_cob_id | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. Set transmission type
    write_sdo(0x1401, 0x02, 0x01, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 3. Clear Mapping
    write_sdo(0x1601, 0x00, 0x00, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 4. Map Control Word (0x6040)
    write_sdo(0x1601, 0x01, 0x60400010, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 5. Map Target Velocity (0x60FF) <-- This enables real-time velocity control
    write_sdo(0x1601, 0x02, 0x60FF0020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 6. Set number of entries to 2
    write_sdo(0x1601, 0x00, 0x02, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 7. Set transmission type and Enable RxPDO2
    write_sdo(0x1401, 0x02, 0xFF, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_sdo(0x1401, 0x01, rxpdo2_cob_id, 4);
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

void CANopenROS2::check_and_clear_error()
{
    // Check Error Register (0x1001)
    int32_t error_register = read_sdo(0x1001, 0x00);
    if (error_register > 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error Register (0x1001): 0x%02X", error_register);
        
        // Read Manufacturer Error Code (0x603F) if supported
        int32_t manufacturer_error = read_sdo(0x603F, 0x00);
        if (manufacturer_error > 0) {
             RCLCPP_ERROR(this->get_logger(), "Manufacturer Error (0x603F): 0x%04X", manufacturer_error);
        }
    }
}

void CANopenROS2::clear_fault()
{
    RCLCPP_INFO(this->get_logger(), "清除故障...");
    
    check_and_clear_error();

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
    
    // 如果状态字读取失败，尝试多次
    int retry_count = 0;
    while (status_word < 0 && retry_count < 3)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        retry_count++;
    }
    
    if (status_word < 0)
    {
        RCLCPP_WARN(this->get_logger(), "无法读取状态字，尝试继续使能流程");
    }
    else
    {
        status_word_ = static_cast<uint16_t>(status_word);
    }
    
    // 检查是否有故障需要清除
    if (status_word_ & 0x0008)  // 故障位
    {
        RCLCPP_WARN(this->get_logger(), "检测到故障，尝试清除...");
        clear_fault();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else if ((status_word_ & 0x004F) == 0x0040) // Operation Inhibit
    {
        RCLCPP_WARN(this->get_logger(), "检测到 Operation Inhibit (0x0040)，尝试复位...");
        // Strict Reset Sequence: 0x0080 -> 0x0006 -> 0x0007 -> 0x000F
        set_control_word(0x0080);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        set_control_word(0x0006);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        set_control_word(0x0007);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        set_control_word(0x000F);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return; // Return early as we've done the sequence
    }
    
    // 执行状态转换序列 - Strict sequence for robustness
    // 1. Shutdown (0x06)
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 2. Switch on (0x07)
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 3. Enable operation (0x0F)
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // 再次读取状态字，确认电机已使能
    status_word = read_sdo(OD_STATUS_WORD, 0x00);
    if (status_word >= 0)
    {
        status_word_ = static_cast<uint16_t>(status_word);
        RCLCPP_INFO(this->get_logger(), "使能后状态字: 0x%04X", status_word_);
        
        // 检查是否成功使能
        if ((status_word_ & 0x006F) == 0x0027)
        {
            RCLCPP_INFO(this->get_logger(), "电机已成功使能 (操作已启用)");
        }
        else if ((status_word_ & 0x006F) == 0x0023)
        {
            RCLCPP_INFO(this->get_logger(), "电机已开启，但未进入操作模式");
        }
        else if ((status_word_ & 0x004F) == 0x0040)
        {
            RCLCPP_WARN(this->get_logger(), "电机仍处于禁止开启状态，可能需要检查硬件或配置");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "无法读取使能后的状态字");
    }
    
    // 使用PDO方式再次尝试使能（更可靠，周期性刷新）
    set_control_word(CONTROL_SHUTDOWN);  // 关机
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    set_control_word(CONTROL_SWITCH_ON);  // 开启
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    set_control_word(CONTROL_ENABLE_OPERATION);  // 使能操作
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
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
    
    // 读取并记录减速比 (0x6091:01 和 0x6091:02) - 仅读取供参考，不修改
    int32_t motor_revolutions = read_sdo(OD_GEAR_RATIO, 0x01);
    int32_t shaft_revolutions = read_sdo(OD_GEAR_RATIO, 0x02);
    
    if (motor_revolutions > 0 && shaft_revolutions > 0)
    {
        float calculated_gear_ratio = static_cast<float>(motor_revolutions) / static_cast<float>(shaft_revolutions);
        RCLCPP_INFO(this->get_logger(), "当前减速比 (0x6091): 电机转数=%d, 轴转数=%d, 计算值=%.2f (配置值=%.2f)", 
                   motor_revolutions, shaft_revolutions, calculated_gear_ratio, gear_ratio_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "无法读取或无效的减速比 (0x6091)，使用配置值: %.2f", gear_ratio_);
    }
    
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
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    write_sdo(OD_PROFILE_VELOCITY, 0x00, velocity_units, 4);
    RCLCPP_INFO(this->get_logger(), "轮廓速度已设置: %.2f°/s", velocity_deg_per_sec);
}

void CANopenROS2::set_profile_acceleration(float acceleration_deg_per_sec2)
{
    int32_t acceleration_units = acceleration_to_units(acceleration_deg_per_sec2);
    write_sdo(OD_PROFILE_ACCELERATION, 0x00, acceleration_units, 4);
    RCLCPP_INFO(this->get_logger(), "轮廓加速度已设置: %.2f°/s²", acceleration_deg_per_sec2);
}

void CANopenROS2::set_profile_deceleration(float deceleration_deg_per_sec2)
{
    int32_t deceleration_units = acceleration_to_units(deceleration_deg_per_sec2);
    write_sdo(OD_PROFILE_DECELERATION, 0x00, deceleration_units, 4);
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

void CANopenROS2::set_target_velocity(int32_t velocity_units_per_sec)
{
    // DSY-C.EDS specifies target velocity at 0x60FF
    write_sdo(OD_TARGET_VELOCITY, 0x00, velocity_units_per_sec, 4);
    RCLCPP_INFO(this->get_logger(), "目标速度已设置(命令单位): %d", velocity_units_per_sec);
}

void CANopenROS2::go_to_position(float angle)
{
    RCLCPP_INFO(this->get_logger(), "移动到位置: %.2f°", angle);
    
    int32_t position = angle_to_position(angle);
    RCLCPP_INFO(this->get_logger(), "目标位置命令单位: %d", position);
    
    // 先使用SDO设置目标位置 (一次性设置)
    write_sdo(OD_TARGET_POSITION, 0x00, position, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 使用PDO进行控制字握手，避免与SDO混用导致状态机混乱
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 6;  // 2 bytes control, 4 bytes position
    
    // 1. 发送 Enable Operation (0x000F) + 目标位置
    // 确保位置数据始终在PDO中，防止数据不一致
    frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
    frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;
    frame.data[2] = position & 0xFF;
    frame.data[3] = (position >> 8) & 0xFF;
    frame.data[4] = (position >> 16) & 0xFF;
    frame.data[5] = (position >> 24) & 0xFF;
    
    write(can_socket_, &frame, sizeof(struct can_frame));
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 短暂等待
    
    // 2. 发送 Enable Operation + New Setpoint (0x001F) + 目标位置 (上升沿触发)
    frame.data[0] = (CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT) & 0xFF;
    write(can_socket_, &frame, sizeof(struct can_frame));
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // 3. 恢复 Enable Operation (0x000F) + 目标位置 (完成握手)
    frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
    write(can_socket_, &frame, sizeof(struct can_frame));
    send_sync_frame();
    
    RCLCPP_INFO(this->get_logger(), "位置命令已通过PDO握手发送");
    
    // 如果目标位置就是当前位置，不需要等待
    int32_t current_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
    int32_t position_diff = (position > current_pos) ? (position - current_pos) : (current_pos - position);
    if (position_diff < 100)  // 如果位置差小于100个命令单位，认为已到达
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
        
        // 检查是否发生故障
        if (status_word & 0x0008 || (status_word & 0x004F) == 0x0040)
        {
             RCLCPP_ERROR(this->get_logger(), "移动过程中检测到故障或禁止开启状态 (0x%04X)", status_word);
             check_and_clear_error(); // 诊断错误
             // 尝试重新使能? 或者直接退出
             break;
        }

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
        RCLCPP_WARN(this->get_logger(), "等待目标位置到达超时 (最终位置差: %d 命令单位, 约 %.2f°)", 
                   final_diff, position_to_angle(final_diff));
    }
}

void CANopenROS2::set_velocity(float velocity_deg_per_sec)
{
    RCLCPP_INFO(this->get_logger(), "设置速度: %.2f°/s", velocity_deg_per_sec);
    
    // 读取当前操作模式
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    
    if (mode != MODE_PROFILE_VELOCITY)
    {
        RCLCPP_WARN(this->get_logger(), "当前不是速度模式，无法设置速度。当前模式: %d", mode);
        return;
    }
    
    // 转换为命令单位
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    
    // 设置目标速度
    write_sdo(0x60FF, 0x00, velocity_units, 4);  // 0x60FF是目标速度对象
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 使能操作
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    
    RCLCPP_INFO(this->get_logger(), "速度已设置: %.2f°/s (命令单位: %d)", velocity_deg_per_sec, velocity_units);
}

void CANopenROS2::set_velocity_pdo(float velocity_deg_per_sec)
{
    RCLCPP_INFO(this->get_logger(), "使用PDO设置速度: %.2f°/s", velocity_deg_per_sec);
    
    // 读取当前操作模式
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "当前操作模式: %d", mode);
    
    // 如果当前不是速度模式，需要先切换到速度模式
    if (mode != MODE_PROFILE_VELOCITY)
    {
        RCLCPP_INFO(this->get_logger(), "当前模式不是速度模式，正在切换到速度模式...");
        
        // 尝试切换到速度模式 (MODE_PROFILE_VELOCITY = 3)
        set_operation_mode(MODE_PROFILE_VELOCITY);
        
        // 再次检查模式
        mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "切换后操作模式: %d", mode);
        
        if (mode != MODE_PROFILE_VELOCITY)
        {
            RCLCPP_ERROR(this->get_logger(), "无法切换到速度模式，当前模式: %d。速度设置可能失败。", mode);
            // 继续尝试设置速度，但可能会失败
        }
    }
    
    // 设置轮廓速度参数（用于速度模式）
    set_profile_velocity(velocity_deg_per_sec);
    
    // 转换为命令单位
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    
    // 确保电机处于操作使能状态
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    uint16_t control_word = CONTROL_ENABLE_OPERATION;
    
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
    
    // 使用RxPDO2发送控制字和目标速度（实时控制）
    struct can_frame frame;
    frame.can_id = COB_RPDO2 + node_id_;
    frame.can_dlc = 6;  // 控制字(2字节) + 目标速度(4字节)
    
    // 控制字 (0x6040) - 2 bytes
    frame.data[0] = control_word & 0xFF;  // 控制字低字节
    frame.data[1] = (control_word >> 8) & 0xFF;  // 控制字高字节
    
    // 目标速度 (0x60FF) - 4 bytes (little-endian)
    frame.data[2] = velocity_units & 0xFF;
    frame.data[3] = (velocity_units >> 8) & 0xFF;
    frame.data[4] = (velocity_units >> 16) & 0xFF;
    frame.data[5] = (velocity_units >> 24) & 0xFF;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "发送速度PDO失败 [节点ID=%d]", node_id_);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "速度PDO已发送 [节点ID=%d]: %.2f°/s (命令单位: %d)", 
                    node_id_, velocity_deg_per_sec, velocity_units);
    }
    
    RCLCPP_INFO(this->get_logger(), "速度命令已发送: %.2f°/s (命令单位: %d)", velocity_deg_per_sec, velocity_units);
}
