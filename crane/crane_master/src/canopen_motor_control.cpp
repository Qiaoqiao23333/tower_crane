#include "crane_master/canopen_ros2_node.hpp"
#include <cmath>

/**
 * @brief 🚀 Initialize CANopen node
 * @details Reset node, set operation mode, configure profile parameters
 */
void CANopenROS2::initialize_node()
{
    // ⏹️ Send NMT stop command
    send_nmt_command(NMT_STOP_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 🔄 Send NMT reset command
    send_nmt_command(NMT_RESET_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // ⚡ Enable motor first, then set operation mode
    // 🔌 Shutdown
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // ⚡ Switch on
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // ✅ Enable operation
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 📊 Read status word to confirm motor is enabled
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "📊 Status word after enable: 0x%04X", status_word);
    
    // 🎛️ Set operation mode to position mode (default)
    write_sdo(OD_OPERATION_MODE, 0x00, MODE_PROFILE_POSITION, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 🔍 Verify operation mode
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "🎛️ Current operation mode: %d", mode);
    
    // 如果操作模式仍然不是位置模式，尝试使用PDO设置
    if (mode != MODE_PROFILE_POSITION)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to set operation mode using SDO, trying PDO");
        
        // 使用PDO设置操作模式
        struct can_frame frame = {};
        frame.can_id = COB_RPDO1 + node_id_;
        frame.can_dlc = 7;  // Default RPDO1: CW(2) + TargetVel(4) + OpMode(1)
        frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
        frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;
        // Target velocity = 0
        frame.data[2] = 0;
        frame.data[3] = 0;
        frame.data[4] = 0;
        frame.data[5] = 0;
        frame.data[6] = MODE_PROFILE_POSITION;
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send PDO operation mode");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "PDO operation mode sent");
        }
        
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 🔍 Verify operation mode again
        mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "🎛️ Operation mode after PDO setting: %d", mode);
    }
    
    // ⚙️ Set electronic gear ratio (from ROS2 parameters: gear_ratio_numerator / denominator)
    set_gear_ratio(gear_ratio_numerator_, gear_ratio_denominator_);
    
    // 📌 Set max profile velocity limit (from ROS2 parameter: max_profile_velocity)
    set_max_profile_velocity(max_profile_velocity_);
    
    // 🏃 Set profile velocity (from ROS2 parameter: profile_velocity)
    set_profile_velocity(profile_velocity_);
    
    // ⬆️ Set profile acceleration (from ROS2 parameter: profile_acceleration)
    set_profile_acceleration(profile_acceleration_);
    
    // ⬇️ Set profile deceleration (from ROS2 parameter: profile_deceleration)
    set_profile_deceleration(profile_deceleration_);
    
    // 🛑 Set quick stop deceleration (from ROS2 parameter: quick_stop_deceleration)
    set_quick_stop_deceleration(quick_stop_deceleration_);
    
    // 📐 Set position range limit (0x607B sub1=max, sub2=min, from ROS2 parameters)
    set_position_range_limit(position_range_limit_max_, position_range_limit_min_);
    
    // 禁用同步生成器
    write_sdo(OD_CYCLE_PERIOD, 0x00, 0, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 设置通信周期为1000微秒
    write_sdo(OD_CYCLE_PERIOD, 0x00, 1000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "✅ Node initialization complete");
}

/**
 * @brief 📋 Configure PDO mapping
 * @details Configure TxPDO1 (status word+position), RxPDO1 (control word+position), RxPDO2 (control word+velocity)
 */
void CANopenROS2::configure_pdo()
{
    RCLCPP_INFO(this->get_logger(), "📋 Configuring PDO mapping...");
    
    // ⏸️ Enter pre-operational mode (0x80)
    // MUST use Pre-Operational, NOT Stop (0x02)!
    // In Pre-Operational: SDO ✅ enabled, PDO ❌ disabled (safe for reconfiguration)
    // In Stopped:         SDO ❌ disabled → all write_sdo() below silently fail!
    send_nmt_command(NMT_ENTER_PRE_OPERATIONAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RCLCPP_INFO(this->get_logger(), "⏸️ Entered pre-operational mode");
    
    // 📤 Configure TxPDO1 (send: status word + actual position)
    RCLCPP_INFO(this->get_logger(), "📤 begin to set TxPDO1");
    
    // 1. Disable TxPDO1
    uint32_t txpdo1_cob_id = COB_TPDO1 + node_id_;
    write_sdo(0x1800, 0x01, txpdo1_cob_id | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. Set transmission type
    write_sdo(0x1800, 0x02, 0x01, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 3. Clear TxPDO1 mapping
    write_sdo(0x1A00, 0x00, 0x00, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 4. Set mapping object: status word
    write_sdo(0x1A00, 0x01, 0x60410010, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 5. Set mapping object: actual position
    write_sdo(0x1A00, 0x02, 0x60640020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 6. Set TxPDO1 mapping object count to 2
    write_sdo(0x1A00, 0x00, 0x02, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 7. Set transmission type and enable TxPDO1
    write_sdo(0x1800, 0x02, 0xFF, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_sdo(0x1800, 0x01, txpdo1_cob_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // ── RPDO1: Force Configure Mapping (CW + TargetVelocity + OpMode = 7 bytes) ──
    // Matches DSY-C default, but we enforce it to fix any persistent dirty state.
    RCLCPP_INFO(this->get_logger(), "📥 Configuring RPDO1 Mapping...");
    uint32_t rxpdo1_cob_id = COB_RPDO1 + node_id_;
    
    // 1. Disable RPDO1
    write_sdo(0x1400, 0x01, rxpdo1_cob_id | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 2. Set transmission type (Synchronous)
    write_sdo(0x1400, 0x02, 0x01, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 3. Clear Mapping
    write_sdo(0x1600, 0x00, 0x00, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 4. Map Objects
    // Sub1: Control Word (0x6040, 16 bit)
    write_sdo(0x1600, 0x01, 0x60400010, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Sub2: Target Velocity (0x60FF, 32 bit)
    write_sdo(0x1600, 0x02, 0x60FF0020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Sub3: Operation Mode (0x6060, 8 bit)
    write_sdo(0x1600, 0x03, 0x60600008, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 5. Set Mapping Count (3)
    write_sdo(0x1600, 0x00, 0x03, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 6. Enable RPDO1
    write_sdo(0x1400, 0x01, rxpdo1_cob_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Read-back RPDO1 mapping for debugging
    int32_t rpdo1_cnt  = read_sdo(0x1600, 0x00);
    int32_t rpdo1_map1 = read_sdo(0x1600, 0x01);
    int32_t rpdo1_map2 = read_sdo(0x1600, 0x02);
    int32_t rpdo1_map3 = read_sdo(0x1600, 0x03);
    RCLCPP_INFO(this->get_logger(), "📋 RPDO1 Active Mapping: Count=%d, [1]=0x%08X, [2]=0x%08X, [3]=0x%08X",
                rpdo1_cnt, rpdo1_map1, rpdo1_map2, rpdo1_map3);
    
    // ── RPDO2: Force Configure Mapping (TargetPosition + ProfileVelocity = 8 bytes) ──
    RCLCPP_INFO(this->get_logger(), "📥 Configuring RPDO2 Mapping...");
    uint32_t rxpdo2_cob_id = 0x300 + node_id_;
    
    // 1. Disable RPDO2
    write_sdo(0x1401, 0x01, rxpdo2_cob_id | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 2. Set transmission type (Synchronous)
    write_sdo(0x1401, 0x02, 0x01, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 3. Clear Mapping
    write_sdo(0x1601, 0x00, 0x00, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 4. Map Objects
    // Sub1: Target Position (0x607A, 32 bit)
    write_sdo(0x1601, 0x01, 0x607A0020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Sub2: Profile Velocity (0x6081, 32 bit)
    write_sdo(0x1601, 0x02, 0x60810020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 5. Set Mapping Count (2)
    write_sdo(0x1601, 0x00, 0x02, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 6. Enable RPDO2
    write_sdo(0x1401, 0x01, rxpdo2_cob_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Read-back RPDO2 mapping for debugging
    int32_t rpdo2_cnt  = read_sdo(0x1601, 0x00);
    int32_t rpdo2_map1 = read_sdo(0x1601, 0x01);
    int32_t rpdo2_map2 = read_sdo(0x1601, 0x02);
    RCLCPP_INFO(this->get_logger(), "📋 RPDO2 Active Mapping: Count=%d, [1]=0x%08X, [2]=0x%08X",
                rpdo2_cnt, rpdo2_map1, rpdo2_map2);
    
    // 🛑 Disable RxPDO3 (COB-ID 0x400 + NodeID) to prevent 0xFF31 RPDO mismatch
    RCLCPP_INFO(this->get_logger(), "🛑 Disabling RxPDO3 and RxPDO4...");
    write_sdo(0x1402, 0x01, (0x400 + node_id_) | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 🛑 Disable RxPDO4 (COB-ID 0x500 + NodeID) to prevent 0xFF31 RPDO mismatch
    write_sdo(0x1403, 0x01, (0x500 + node_id_) | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 🛑 Disable TxPDO2, TxPDO3, TxPDO4 just in case
    write_sdo(0x1801, 0x01, (0x280 + node_id_) | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write_sdo(0x1802, 0x01, (0x380 + node_id_) | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write_sdo(0x1803, 0x01, (0x480 + node_id_) | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    RCLCPP_INFO(this->get_logger(), "✅ PDO setting finished");
}

void CANopenROS2::start_node()
{
    send_nmt_command(NMT_START_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 获取实际位置，用于接下来的RPDO1
    position_ = read_sdo(OD_ACTUAL_POSITION, 0x00);
    RCLCPP_INFO(this->get_logger(), "📍 当前实际位置: %d", position_);
    
    // ── 在发送第一个 SYNC 之前预充 RPDO 缓冲区 ──────────────────
    // 驱动器会在每个SYNC上验证RPDO数据。如果不提前把数据写进去，
    // 就会触发 0xFF31 报错。调用 set_control_word 刚好可以同时预充 RPDO1 和 RPDO2。
    set_control_word(CONTROL_ENABLE_OPERATION);
    
    // 发送同步帧
    send_sync_frame();
    
    RCLCPP_INFO(this->get_logger(), "✅ 节点启动完成");
}

void CANopenROS2::set_immediate_effect(bool immediate)
{
    RCLCPP_INFO(this->get_logger(), "⚡ setting %s effect", immediate ? "immediate" : "non-immediate");
    
    // Read current control word
    int32_t controlword = read_sdo(OD_CONTROL_WORD, 0x00);
    
    if (immediate)
    {
        controlword |= (1 << 5);  // Set bit 5 to 1 (immediate effect)
    }
    else
    {
        controlword &= ~(1 << 5);  // Set bit 5 to 0 (non-immediate effect)
    }
    
    // Write new control word
    write_sdo(OD_CONTROL_WORD, 0x00, controlword, 2);
    
    RCLCPP_INFO(this->get_logger(), "🎮 Control word updated to: 0x%04X", controlword);
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
        
        // Clear the fault via PDO (SDO writes to 0x6040 are silently ignored
        // while PDO-mapped objects are active on the DSY-C drive).
        set_control_word(CONTROL_FAULT_RESET);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Transition to clean state: Shutdown
        set_control_word(CONTROL_SHUTDOWN);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Verify error is cleared
        error_register = read_sdo(0x1001, 0x00);
        if (error_register > 0)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ Error still present after reset: 0x%02X", error_register);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "✅ Error cleared");
        }
    }
}

void CANopenROS2::clear_fault()
{
    RCLCPP_INFO(this->get_logger(), "🚨 Clearing fault...");
    
    check_and_clear_error();

    // Send fault reset command
    set_control_word(CONTROL_FAULT_RESET);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Enter ready state
    set_control_word(CONTROL_SHUTDOWN);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "✅ Fault cleared");
}

void CANopenROS2::enable_motor()
{
    RCLCPP_INFO(this->get_logger(), "⚡ Enabling motor...");
    
    // Read current status word
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "📊 Current status word: 0x%04X", status_word);
    
    // If status word read fails, retry multiple times
    int retry_count = 0;
    while (status_word < 0 && retry_count < 3)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        retry_count++;
    }
    
    if (status_word < 0)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unable to read status word, attempting to continue enable process");
    }
    else
    {
        status_word_ = static_cast<uint16_t>(status_word);
    }
    
    // Check if there are faults to clear
    if (status_word_ & 0x0008)  // Fault bit
    {
        RCLCPP_WARN(this->get_logger(), "🚨 Fault detected, attempting to clear...");
        clear_fault();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    else if ((status_word_ & 0x004F) == 0x0040) // Switch on disabled (normal after NMT Reset)
    {
        RCLCPP_INFO(this->get_logger(), "ℹ️ Switch on disabled (0x0040) – normal post-reset state, proceeding with enable...");
    }
    
    // ── PDO-only CiA 402 state machine transition ─────────────────
    // Control word (0x6040) is mapped in RxPDO1.  The DSY-C drive gives
    // RPDO-written values priority over SDO for PDO-mapped objects, so
    // SDO writes to 0x6040 are silently ignored while PDOs are active.
    // Use set_control_word() (RPDO1) exclusively for reliable transitions.
    
    // 1. Shutdown (0x06) → "Ready to switch on"
    set_control_word(CONTROL_SHUTDOWN);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. Switch on (0x07) → "Switched on"
    set_control_word(CONTROL_SWITCH_ON);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 3. Enable operation (0x0F) → "Operation enabled"
    set_control_word(CONTROL_ENABLE_OPERATION);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Confirm final state
    status_word = read_sdo(OD_STATUS_WORD, 0x00);
    if (status_word >= 0)
    {
        status_word_ = static_cast<uint16_t>(status_word);
        RCLCPP_INFO(this->get_logger(), "📊 Status word after enable: 0x%04X", status_word_);
        
        if ((status_word_ & 0x006F) == 0x0027)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Motor successfully enabled (operation enabled)");
        }
        else if ((status_word_ & 0x006F) == 0x0023)
        {
            RCLCPP_INFO(this->get_logger(), "⚠️ Motor is on but not in operation mode");
        }
        else if ((status_word_ & 0x004F) == 0x0040)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ Motor still in inhibit state, may need to check hardware or configuration");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unable to read status word after enable");
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Motor enabled");
}

void CANopenROS2::stop_motor()
{
    RCLCPP_INFO(this->get_logger(), "✋🏻 Stopping motor...");
    
    // Set target velocity to 0
    set_target_velocity(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Disable operation
    set_control_word(CONTROL_SWITCH_ON);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Turn off motor
    set_control_word(CONTROL_SHUTDOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Disable voltage
    set_control_word(CONTROL_DISABLE_VOLTAGE);
    
    RCLCPP_INFO(this->get_logger(), "✅ Motor stopped");
}

void CANopenROS2::initialize_motor()
{
    // Initialize node
    initialize_node();
    
    // Configure PDO mapping
    configure_pdo();
    
    // Wait for a period
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Start node
    start_node();
    
    // Set immediate effect
    set_immediate_effect(true);
    
    // Clear fault
    clear_fault();
    
    // Enable motor
    enable_motor();
    
    // Read and record gear ratio (0x6091:01 and 0x6091:02) - read only for reference, do not modify
    int32_t motor_revolutions = read_sdo(OD_GEAR_RATIO, 0x01);
    int32_t shaft_revolutions = read_sdo(OD_GEAR_RATIO, 0x02);
    
    if (motor_revolutions > 0 && shaft_revolutions > 0)
    {
        RCLCPP_INFO(this->get_logger(), "⚙️ Current drive gear ratio (0x6091): motor revolutions=%d, shaft revolutions=%d", 
                   motor_revolutions, shaft_revolutions);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "⚠️ Unable to read current drive gear ratio (0x6091)");
    }
    
    // Ensure motor stays still: set target velocity to 0 and target position to current position
    RCLCPP_INFO(this->get_logger(), "🛑 Ensuring motor stays still after initialization...");
    
    // 1. Set target velocity to 0 (important if motor was in velocity mode)
    set_target_velocity(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. Ensure we're in position mode
    int32_t current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    if (current_mode != MODE_PROFILE_POSITION)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Motor not in position mode (current: %d), switching to position mode...", current_mode);
        set_operation_mode(MODE_PROFILE_POSITION);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    // 3. Read current position and set target position to current position (keep motor still)
    int32_t current_position = read_sdo(OD_ACTUAL_POSITION, 0x00);
    if (current_position >= 0)
    {
        write_sdo(OD_TARGET_POSITION, 0x00, current_position, 4);
        float current_angle = position_to_angle(current_position);
        RCLCPP_INFO(this->get_logger(), "📍 Motor initialized at current position: %.2f° (keeping still)", current_angle);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unable to read current position, setting target to 0");
        write_sdo(OD_TARGET_POSITION, 0x00, 0, 4);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "✅ Motor initialization complete");
}

void CANopenROS2::set_operation_mode(uint8_t mode)
{
    RCLCPP_INFO(this->get_logger(), "🫵🏻 Starting to switch operation mode to: %d", mode);
    
    // Step 1: Read current status word and mode
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    int32_t current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "👉🏻 Current status word: 0x%04X, current mode: %d", status_word, current_mode);
    
    if (current_mode == mode)
    {
        RCLCPP_INFO(this->get_logger(), "✅ Already in target mode: %d", mode);
        current_op_mode_ = mode;
        return;
    }
    
    uint8_t old_mode = current_op_mode_;
    
    // Update cached mode FIRST — every subsequent set_control_word() + SYNC
    // will carry the new OpMode in the RPDO1 byte (0x6060 is PDO-mapped).
    // The DSY-C drive prioritises RPDO-written values over SDO writes for
    // PDO-mapped objects, so this is the primary mode-switching mechanism.
    current_op_mode_ = mode;
    
    // Step 2: State machine transition via PDO
    // (SDO writes to 0x6040 are ignored while RPDO1 is active on DSY-C)
    set_control_word(CONTROL_SHUTDOWN);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    set_control_word(CONTROL_SWITCH_ON);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Belt-and-suspenders: also try SDO for 0x6060
    write_sdo(OD_OPERATION_MODE, 0x00, mode, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // Verify mode
    current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), " 🫶🏻Operation mode after setting: %d", current_mode);
    
    if (current_mode != mode)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Mode read-back mismatch, retrying...");
        
        // Send another RPDO1+SYNC cycle with the new mode
        set_control_word(CONTROL_ENABLE_OPERATION);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        write_sdo(OD_OPERATION_MODE, 0x00, mode, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "📊 Operation mode after retry: %d", current_mode);
    }
    
    if (current_mode == mode)
    {
        RCLCPP_INFO(this->get_logger(), "✅ Operation mode set successfully, re-enabling motor");
        
        set_control_word(CONTROL_SHUTDOWN);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        set_control_word(CONTROL_SWITCH_ON);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        set_control_word(CONTROL_ENABLE_OPERATION);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "👉🏻 Final status word: 0x%04X", status_word);
        RCLCPP_INFO(this->get_logger(), "✅ Operation mode switch successful: %d", mode);
    }
    else
    {
        current_op_mode_ = old_mode;  // Rollback on failure
        RCLCPP_ERROR(this->get_logger(), "❌ Operation mode switch failed, current mode: %d, expected mode: %d", 
                    current_mode, mode);
        RCLCPP_ERROR(this->get_logger(), "⚠️ Please check if motor supports mode %d, or refer to motor documentation", mode);
    }
}

void CANopenROS2::set_profile_velocity(float velocity_rev_per_sec)
{
    int32_t velocity_units = velocity_to_units(velocity_rev_per_sec);
    write_sdo(OD_PROFILE_VELOCITY, 0x00, velocity_units, 4);
    RCLCPP_INFO(this->get_logger(), "🏃 Profile velocity set: %.2f r/s (%d units)", velocity_rev_per_sec, velocity_units);
}

void CANopenROS2::set_profile_acceleration(float acceleration_rev_per_sec2)
{
    int32_t acceleration_units = acceleration_to_units(acceleration_rev_per_sec2);
    write_sdo(OD_PROFILE_ACCELERATION, 0x00, acceleration_units, 4);
    RCLCPP_INFO(this->get_logger(), "⬆️ Profile acceleration set: %.2f r/s² (%d units)", acceleration_rev_per_sec2, acceleration_units);
}

void CANopenROS2::set_profile_deceleration(float deceleration_rev_per_sec2)
{
    int32_t deceleration_units = acceleration_to_units(deceleration_rev_per_sec2);
    write_sdo(OD_PROFILE_DECELERATION, 0x00, deceleration_units, 4);
    RCLCPP_INFO(this->get_logger(), "⬇️ Profile deceleration set: %.2f r/s² (%d units)", deceleration_rev_per_sec2, deceleration_units);
}

void CANopenROS2::set_profile_parameters(float velocity_rev_per_sec, float acceleration_rev_per_sec2, float deceleration_rev_per_sec2)
{
    set_profile_velocity(velocity_rev_per_sec);
    set_profile_acceleration(acceleration_rev_per_sec2);
    set_profile_deceleration(deceleration_rev_per_sec2);
    RCLCPP_INFO(this->get_logger(), "📏 Profile parameters set - velocity: %.2f r/s, acceleration: %.2f r/s², deceleration: %.2f r/s²", 
               velocity_rev_per_sec, acceleration_rev_per_sec2, deceleration_rev_per_sec2);
}

void CANopenROS2::set_control_word(uint16_t control_word)
{
    // ── RPDO1 (default mapping): CW(2) + TargetVelocity(4) + OpMode(1) = 7 bytes ──
    struct can_frame frame = {};
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 7;
    frame.data[0] = control_word & 0xFF;
    frame.data[1] = (control_word >> 8) & 0xFF;
    // Target velocity = 0 (hold still)
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = current_op_mode_;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 发送RPDO1控制字失败");
    }
    
    // ── RPDO2 (default mapping): TargetPosition(4) + ProfileVelocity(4) = 8 bytes ──
    struct can_frame rpdo2 = {};
    rpdo2.can_id = COB_RPDO2 + node_id_;
    rpdo2.can_dlc = 8;
    // Target position = current cached position (hold still)
    rpdo2.data[0] = position_ & 0xFF;
    rpdo2.data[1] = (position_ >> 8) & 0xFF;
    rpdo2.data[2] = (position_ >> 16) & 0xFF;
    rpdo2.data[3] = (position_ >> 24) & 0xFF;
    // Profile velocity = 0 (hold still)
    rpdo2.data[4] = 0;
    rpdo2.data[5] = 0;
    rpdo2.data[6] = 0;
    rpdo2.data[7] = 0;
    write(can_socket_, &rpdo2, sizeof(struct can_frame));
    
    RCLCPP_INFO(this->get_logger(), "🎮 控制字已发送: 0x%04X (RPDO1 & RPDO2 已同步)", control_word);
}

void CANopenROS2::set_target_velocity(int32_t velocity_units_per_sec)
{
    // DSY-C.EDS specifies target velocity at 0x60FF
    write_sdo(OD_TARGET_VELOCITY, 0x00, velocity_units_per_sec, 4);
    RCLCPP_INFO(this->get_logger(), "🎯 Target velocity set (command units): %d", velocity_units_per_sec);
}

void CANopenROS2::go_to_position(float angle)
{
    RCLCPP_INFO(this->get_logger(), "📍 Moving to position: %.2f°", angle);
    
    // 1. Ensure we're in profile position mode
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    if (mode != MODE_PROFILE_POSITION) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Wrong mode (%d), forcing switch to profile position mode (1)", mode);
        set_operation_mode(MODE_PROFILE_POSITION);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    current_op_mode_ = MODE_PROFILE_POSITION;

    int32_t position = angle_to_position(angle);
    position_ = position;  // Update cached position for set_control_word()
    
    // 2. Write target position via SDO as well (belt-and-suspenders)
    write_sdo(OD_TARGET_POSITION, 0x00, position, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Helper lambda: send RPDO1 + RPDO2 + SYNC
    // Default RPDO1: CW(2) + TargetVelocity(4) + OpMode(1) = 7 bytes
    // Default RPDO2: TargetPosition(4) + ProfileVelocity(4) = 8 bytes
    int32_t prof_vel = velocity_to_units(profile_velocity_);
    auto send_rpdo_and_sync = [&](uint16_t control_word) {
        // RPDO1: CW + TargetVelocity=0 + OpMode=1 (position mode)
        struct can_frame rpdo1 = {};
        rpdo1.can_id = COB_RPDO1 + node_id_;
        rpdo1.can_dlc = 7;
        rpdo1.data[0] = control_word & 0xFF;
        rpdo1.data[1] = (control_word >> 8) & 0xFF;
        // Target velocity = 0 (position mode uses profile velocity from RPDO2)
        rpdo1.data[2] = 0;
        rpdo1.data[3] = 0;
        rpdo1.data[4] = 0;
        rpdo1.data[5] = 0;
        rpdo1.data[6] = MODE_PROFILE_POSITION;
        write(can_socket_, &rpdo1, sizeof(struct can_frame));
        
        // RPDO2: TargetPosition + ProfileVelocity
        struct can_frame rpdo2 = {};
        rpdo2.can_id = COB_RPDO2 + node_id_;
        rpdo2.can_dlc = 8;
        rpdo2.data[0] = position & 0xFF;
        rpdo2.data[1] = (position >> 8) & 0xFF;
        rpdo2.data[2] = (position >> 16) & 0xFF;
        rpdo2.data[3] = (position >> 24) & 0xFF;
        rpdo2.data[4] = prof_vel & 0xFF;
        rpdo2.data[5] = (prof_vel >> 8) & 0xFF;
        rpdo2.data[6] = (prof_vel >> 16) & 0xFF;
        rpdo2.data[7] = (prof_vel >> 24) & 0xFF;
        write(can_socket_, &rpdo2, sizeof(struct can_frame));
        
        // SYNC: drive latches RPDO data on this pulse
        send_sync_frame();
    };
    
    // 3. CiA 402 "New Set Point" handshake (profile position mode)
    //    The drive requires a rising edge on bit 4 to accept a new target position.
    
    // Step A: Send Enable Operation (0x0F) + target position, then SYNC
    send_rpdo_and_sync(CONTROL_ENABLE_OPERATION);  // 0x0F
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // Step B: Rising edge — set New Set Point + Change Immediately (0x3F), then SYNC
    //         0x3F = Enable Operation (0x0F) | New Set Point (0x10) | Change Immediately (0x20)
    send_rpdo_and_sync(CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT | (1 << 5));  // 0x3F
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // Step C: Wait for drive to acknowledge (Status Word bit 12 = Set-point Acknowledge)
    int ack_retry = 0;
    bool acknowledged = false;
    while (ack_retry < 50) {
        int32_t sw = read_sdo(OD_STATUS_WORD, 0x00);
        if (sw & 0x1000) {  // Bit 12: Set-point Acknowledge
            acknowledged = true;
            RCLCPP_DEBUG(this->get_logger(), "✅ Drive acknowledged new set point (bit 12)");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ack_retry++;
    }
    
    if (!acknowledged) {
        RCLCPP_ERROR(this->get_logger(), "❌ Set-point acknowledge timeout — drive did not accept position command!");
        return;
    }
    
    // Step D: Clear New Set Point bit to complete handshake (0x0F), then SYNC
    send_rpdo_and_sync(CONTROL_ENABLE_OPERATION);  // 0x0F
    
    RCLCPP_INFO(this->get_logger(), "📍 Position command accepted, target: %d units (%.2f°)", position, angle);
    
    // 4. Monitor until target is reached
    //    IMPORTANT: Keep sending periodic RPDO + SYNC so the drive's synchronous
    //    position controller continues to run.  RPDOs are type 0x01 (synchronous),
    //    so the drive only updates its control loop on each SYNC pulse.  Without
    //    periodic SYNCs the drive never completes the deceleration phase and the
    //    motor coasts past the target indefinitely.
    int retry = 0;
    const int max_retries = 100;  // 100 × 200ms = 20s timeout
    while (retry < max_retries)
    {
        // Refresh RPDO1 (control word + target position) + RPDO2 (control word +
        // velocity=0) and send SYNC so the drive executes another control cycle.
        send_rpdo_and_sync(CONTROL_ENABLE_OPERATION);  // 0x0F
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        int32_t sw = read_sdo(OD_STATUS_WORD, 0x00);
        
        // Check for fault during movement
        if (sw & 0x0008) {
            RCLCPP_ERROR(this->get_logger(), "❌ Fault detected during movement (status: 0x%04X)", sw);
            check_and_clear_error();
            break;
        }
        
        int32_t actual_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
        int32_t current_diff = std::abs(position - actual_pos);
        
        // Target Reached (bit 10) AND position close enough
        if ((sw & 0x0400) && current_diff < 100)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Target position reached and settled! Error: %d units", current_diff);
            break;
        }
        
        if (retry % 10 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "⏳ Waiting for target... (status: 0x%04X, diff: %d, retry %d/%d)",
                         sw, current_diff, retry, max_retries);
        }
        retry++;
    }
    
    if (retry >= max_retries) {
        int32_t final_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
        int32_t final_diff = std::abs(position - final_pos);
        RCLCPP_WARN(this->get_logger(), "⏰ Timeout waiting for target position (final diff: %d units, ~%.2f°)",
                   final_diff, position_to_angle(final_diff));
        // Safety: halt the motor so it does not run indefinitely
        RCLCPP_WARN(this->get_logger(), "🛑 Halting motor after timeout");
        send_rpdo_and_sync(CONTROL_ENABLE_OPERATION | (1 << 8));  // Bit 8 = Halt
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        send_rpdo_and_sync(CONTROL_ENABLE_OPERATION);  // Clear halt, hold position
    }
}

void CANopenROS2::set_velocity(float velocity_deg_per_sec)
{
    RCLCPP_INFO(this->get_logger(), "🏃 Set velocity: %.2f°/s", velocity_deg_per_sec);
    
    // Read current operation mode
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    
    if (mode != MODE_PROFILE_VELOCITY)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Currently not in velocity mode, cannot set velocity. Current mode: %d", mode);
        return;
    }
    
    // Convert to command units
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    
    // Set target velocity
    write_sdo(0x60FF, 0x00, velocity_units, 4);  // 0x60FF is target velocity object
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Enable operation
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    
    RCLCPP_INFO(this->get_logger(), "✅ Velocity set: %.2f°/s (command units: %d)", velocity_deg_per_sec, velocity_units);
}

void CANopenROS2::set_velocity_pdo(float velocity_deg_per_sec)
{
    RCLCPP_INFO(this->get_logger(), "🚀 Set velocity using PDO: %.2f°/s", velocity_deg_per_sec);
    
    // Read current operation mode
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "🎛️ Current operation mode: %d", mode);
    
    // If currently not in velocity mode, need to switch to velocity mode first
    if (mode != MODE_PROFILE_VELOCITY)
    {
        RCLCPP_INFO(this->get_logger(), "🎛️ Current mode is not velocity mode, switching to velocity mode...");
        
        // Try to switch to velocity mode (MODE_PROFILE_VELOCITY = 3)
        set_operation_mode(MODE_PROFILE_VELOCITY);
        
        // Check mode again
        mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "📊 Operation mode after switch: %d", mode);
        
        if (mode != MODE_PROFILE_VELOCITY)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Unable to switch to velocity mode, current mode: %d. Velocity setting may fail.", mode);
            // Continue trying to set velocity, but may fail
        }
    }
    current_op_mode_ = MODE_PROFILE_VELOCITY;
    
    // Set profile velocity parameter (for velocity mode)
    set_profile_velocity(velocity_deg_per_sec);
    
    // Convert to command units
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    
    // Ensure motor is in operation enabled state (use PDO since SDO writes
    // to 0x6040 are ignored while RPDO1 is active on DSY-C)
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    uint16_t control_word = CONTROL_ENABLE_OPERATION;
    
    if ((status_word & 0x006F) != 0x0027)  // If not in "operation enabled" state
    {
        RCLCPP_INFO(this->get_logger(), "⚡ Motor not in operation state, enabling...");
        set_control_word(CONTROL_SHUTDOWN);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        set_control_word(CONTROL_SWITCH_ON);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        set_control_word(CONTROL_ENABLE_OPERATION);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // ── RPDO1 (default mapping): CW(2) + TargetVelocity(4) + OpMode=3(1) = 7 bytes ──
    struct can_frame frame = {};
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 7;
    frame.data[0] = control_word & 0xFF;
    frame.data[1] = (control_word >> 8) & 0xFF;
    frame.data[2] = velocity_units & 0xFF;
    frame.data[3] = (velocity_units >> 8) & 0xFF;
    frame.data[4] = (velocity_units >> 16) & 0xFF;
    frame.data[5] = (velocity_units >> 24) & 0xFF;
    frame.data[6] = MODE_PROFILE_VELOCITY;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to send velocity PDO [node %d]", node_id_);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "📤 Velocity PDO sent [node %d]: %.2f°/s (command units: %d)", 
                    node_id_, velocity_deg_per_sec, velocity_units);
        
        // ── RPDO2: TargetPos=current(4) + ProfileVel=0(4) to prevent 0xFF31 ──
        struct can_frame rpdo2 = {};
        rpdo2.can_id = COB_RPDO2 + node_id_;
        rpdo2.can_dlc = 8;
        rpdo2.data[0] = position_ & 0xFF;
        rpdo2.data[1] = (position_ >> 8) & 0xFF;
        rpdo2.data[2] = (position_ >> 16) & 0xFF;
        rpdo2.data[3] = (position_ >> 24) & 0xFF;
        rpdo2.data[4] = 0;
        rpdo2.data[5] = 0;
        rpdo2.data[6] = 0;
        rpdo2.data[7] = 0;
        write(can_socket_, &rpdo2, sizeof(struct can_frame));
        
        send_sync_frame();
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Velocity command sent: %.2f°/s (command units: %d)", velocity_deg_per_sec, velocity_units);
}

/**
 * @brief 📐 Set position range limit (0x607B)
 * @details Writes sub-index 1 (maximum) and sub-index 2 (minimum) position range limits
 *          to the drive object dictionary. Units are position command units (same as 0x607A).
 * @param max_val  Maximum position range limit [position units] → 0x607B:01
 * @param min_val  Minimum position range limit [position units] → 0x607B:02
 */
void CANopenROS2::set_position_range_limit(int32_t max_val, int32_t min_val)
{
    // Write maximum position range limit → 0x607B sub-index 1
    write_sdo(OD_POSITION_RANGE_LIMIT, 0x01, max_val, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Write minimum position range limit → 0x607B sub-index 2
    write_sdo(OD_POSITION_RANGE_LIMIT, 0x02, min_val, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RCLCPP_INFO(this->get_logger(),
        "👌🏼 Position range limit set: max=%d, min=%d (0x607B:01/02)",
        max_val, min_val);
}

/**
 * @brief 🛑 Set quick stop deceleration (0x6085)
 * @details Writes the quick stop deceleration value to the drive object dictionary.
 *          This value is used when a quick stop command is issued (e.g. fault or NMT stop).
 *          Unit: driving-shaft (output) revolutions/s².
 *          1 command unit = 1 driving-shaft revolution (same 1:1 scale as 0x607A / angle_to_position).
 *          The drive's 0x6091 gear ratio handles the internal motor-side scaling.
 * @param deceleration_rev_per_sec2  Quick stop deceleration [r/s²] (output shaft)
 */
void CANopenROS2::set_quick_stop_deceleration(float deceleration_rev_per_sec2)
{
    // Same 1:1 scale as acceleration_to_units:
    // the drive's 0x6091 gear ratio handles motor-side scaling internally.
    int32_t decel_units = acceleration_to_units(deceleration_rev_per_sec2);

    write_sdo(OD_QUICK_STOP_DECEL, 0x00, decel_units, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RCLCPP_INFO(this->get_logger(),
        "🛑 Quick stop deceleration set: %.2f r/s² (%d command units/s²) (0x6085)",
        deceleration_rev_per_sec2, decel_units);
}

/**
 * @brief 📌 Set maximum profile velocity (0x607F)
 * @details Writes the upper velocity limit to the drive. The drive will clamp
 *          any profile velocity (0x6081) command to this value.
 *          Unit accepted here: r/s (revolutions per second of the output shaft).
 *          Conversion: command_units/s = rev_per_sec × (gear_ratio_numerator_ / gear_ratio_denominator_)
 * @param velocity_rev_per_sec  Max profile velocity [r/s]
 */
void CANopenROS2::set_max_profile_velocity(float velocity_rev_per_sec)
{
    // Same 1:1 scale as set_profile_velocity / velocity_to_units:
    // the drive's 0x6091 gear ratio handles motor-side scaling internally.
    int32_t velocity_units = velocity_to_units(velocity_rev_per_sec);

    write_sdo(OD_MAX_PROFILE_VELOCITY, 0x00, velocity_units, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RCLCPP_INFO(this->get_logger(),
        "📌 Max profile velocity set: %.2f r/s (%d command units/s) (0x607F)",
        velocity_rev_per_sec, velocity_units);
}

/**
 * @brief ⚙️ Set electronic gear ratio (0x6091)
 * @details Writes the electronic gear ratio to the drive.
 *          CiA 402: position_demand is scaled by (motor_shaft_revolutions / drive_shaft_revolutions).
 *          Example: 10:1 physical gear → numerator=10, denominator=1.
 *
 * @param numerator    0x6091:01  motor shaft revolutions  (uint32, must be ≥ 1)
 * @param denominator  0x6091:02  drive shaft revolutions  (uint32, must be ≥ 1)
 */
void CANopenROS2::set_gear_ratio(uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0)
    {
        RCLCPP_ERROR(this->get_logger(),
            "❌ gear_ratio_denominator must not be 0 – skipping 0x6091 write");
        return;
    }

    // Sub-index 1: motor shaft revolutions
    write_sdo(OD_GEAR_RATIO, 0x01, static_cast<int32_t>(numerator), 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Sub-index 2: drive shaft revolutions
    write_sdo(OD_GEAR_RATIO, 0x02, static_cast<int32_t>(denominator), 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RCLCPP_INFO(this->get_logger(),
        "⚙️ Electronic gear ratio set: %u / %u (0x6091:01 / 0x6091:02)",
        numerator, denominator);
}
