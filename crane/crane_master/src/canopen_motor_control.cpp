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
    
    // If SDO setting fails, try using PDO
    if (mode != MODE_PROFILE_POSITION)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to set operation mode using SDO, trying PDO");
        
        // 📤 Use PDO to set operation mode
        struct can_frame frame;
        frame.can_id = COB_RPDO1 + node_id_;
        frame.can_dlc = 3;  // Control word (2 bytes) + operation mode (1 byte)
        frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
        frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;
        frame.data[2] = MODE_PROFILE_POSITION;
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to send PDO operation mode");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "📤 PDO operation mode sent");
        }
        
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 🔍 Verify operation mode again
        mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "🎛️ Operation mode after PDO setting: %d", mode);
    }
    
    // ⚖️ Set position factor (from ROS2 parameters: position_factor_numerator / denominator)
    set_position_factor(position_factor_numerator_, position_factor_denominator_);
    
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
    
    // ⏸️ Disable sync generator
    write_sdo(OD_CYCLE_PERIOD, 0x00, 0, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // ⏱️ Set communication cycle to 1000 microseconds
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
    
    // ⏸️ Enter pre-operation mode
    send_nmt_command(NMT_STOP_REMOTE_NODE);
    RCLCPP_INFO(this->get_logger(), "⏸️ already in pre-operation mode");
    
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
    
    // Configure RxPDO1
    RCLCPP_INFO(this->get_logger(), "📥 begin to set  RxPDO1 (receive : control word  + target position)");
    
    // 1. Disable RxPDO1
    uint32_t rxpdo1_cob_id = COB_RPDO1 + node_id_;
    write_sdo(0x1400, 0x01, rxpdo1_cob_id | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. Set transmission type
    write_sdo(0x1400, 0x02, 0x01, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 3. Clear RxPDO1 mapping
    write_sdo(0x1600, 0x00, 0x00, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 4. Set mapping object: control word
    write_sdo(0x1600, 0x01, 0x60400010, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 5. Set mapping object: target position
    write_sdo(0x1600, 0x02, 0x607A0020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 6. Set RxPDO1 mapping object count to 2
    write_sdo(0x1600, 0x00, 0x02, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 7. Set transmission type and enable RxPDO1
    write_sdo(0x1400, 0x02, 0xFF, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_sdo(0x1400, 0x01, rxpdo1_cob_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure RxPDO2 for Velocity Control
    RCLCPP_INFO(this->get_logger(), "🏃 Configure RxPDO2 (for velocity control)");
    
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
    
    RCLCPP_INFO(this->get_logger(), "✅ PDO setting finished");
}

void CANopenROS2::start_node()
{
    RCLCPP_INFO(this->get_logger(), "▶️ launching Node...");
    
    // Send NMT start command
    send_nmt_command(NMT_START_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Get actual position
    int32_t actual_position = read_sdo(OD_ACTUAL_POSITION, 0x00);
    float actual_angle = position_to_angle(actual_position);
    RCLCPP_INFO(this->get_logger(), "📍 actual position: %.2f°", actual_angle);
    
    // Send sync frame
    send_sync_frame();
    
    RCLCPP_INFO(this->get_logger(), "✅ Node launch succeeded");
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
    }
}

void CANopenROS2::clear_fault()
{
    RCLCPP_INFO(this->get_logger(), "🚨 Clearing fault...");
    
    check_and_clear_error();

    // Send fault reset command
    set_control_word(CONTROL_FAULT_RESET);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Enter ready state
    set_control_word(CONTROL_SHUTDOWN);
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
    } else if ((status_word_ & 0x004F) == 0x0040) // Operation Inhibit
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Operation Inhibit (0x0040) detected, attempting reset...");
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
    
    // Execute state transition sequence - Strict sequence for robustness
    // 1. Shutdown (0x06)
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 2. Switch on (0x07)
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 3. Enable operation (0x0F)
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // Read status word again to confirm motor is enabled
    status_word = read_sdo(OD_STATUS_WORD, 0x00);
    if (status_word >= 0)
    {
        status_word_ = static_cast<uint16_t>(status_word);
        RCLCPP_INFO(this->get_logger(), "📊 Status word after enable: 0x%04X", status_word_);
        
        // Check if enable was successful
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
    
    // Try enabling again using PDO method (more reliable, periodic refresh)
    set_control_word(CONTROL_SHUTDOWN);  // Shutdown
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    set_control_word(CONTROL_SWITCH_ON);  // Switch on
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    set_control_word(CONTROL_ENABLE_OPERATION);  // Enable operation
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    RCLCPP_INFO(this->get_logger(), "👍🏻 Motor enabled");
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
        float calculated_gear_ratio = static_cast<float>(motor_revolutions) / static_cast<float>(shaft_revolutions);
        RCLCPP_INFO(this->get_logger(), "⚙️ Current gear ratio (0x6091): motor revolutions=%d, shaft revolutions=%d, calculated value=%.2f (configured value=%.2f)", 
                   motor_revolutions, shaft_revolutions, calculated_gear_ratio, gear_ratio_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "⚠️ Unable to read or invalid gear ratio (0x6091), using configured value: %.2f", gear_ratio_);
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
    
    // Step 1: First read current status word and mode
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    int32_t current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "👉🏻 Current status word: 0x%04X, current mode: %d", status_word, current_mode);
    
    // If already in target mode, return directly
    if (current_mode == mode)
    {
        RCLCPP_INFO(this->get_logger(), "✅ Already in target mode: %d", mode);
        return;
    }
    
    // Step 2: Ensure motor is in "Ready to switch on" state (0x0021) or "Switched on" (0x0023)
    // This is the state required to switch operation mode
    if ((status_word & 0x006F) != 0x0021 && (status_word & 0x006F) != 0x0023)
    {
        RCLCPP_INFO(this->get_logger(), "⚠️ Motor is not in state that allows mode switching, converting state...");
        
        // First disable operation
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Convert to "Ready to switch on"
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Check status
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "👉🏻 Status word after state conversion: 0x%04X", status_word);
    }
    
    // Step 3: Set operation mode (must be in "Ready to switch on" or "Switched on" state)
    RCLCPP_INFO(this->get_logger(), "✍🏻 Set operation mode to: %d", mode);
    write_sdo(OD_OPERATION_MODE, 0x00, mode, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // Check if operation mode was set successfully
    current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), " 🫶🏻Operation mode after setting: %d", current_mode);
    
    // If SDO setting fails, try using PDO
    if (current_mode != mode)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ SDO mode setting failed, trying PDO");
        
        // Send operation mode using PDO (via RPDO1, if operation mode is mapped)
        // Note: This requires RPDO1 mapping to include operation mode object
        struct can_frame frame;
        frame.can_id = COB_RPDO1 + node_id_;
        frame.can_dlc = 3;  // Control word (2 bytes) + operation mode (1 byte)
        frame.data[0] = CONTROL_SWITCH_ON & 0xFF;  // Control word low byte
        frame.data[1] = (CONTROL_SWITCH_ON >> 8) & 0xFF;  // Control word high byte
        frame.data[2] = mode;  // Operation mode
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to send PDO operation mode");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "📤 PDO operation mode sent: %d", mode);
        }
        
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Check operation mode again
        current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "📊 Operation mode after PDO setting: %d", current_mode);
    }
    
    // Step 4: If mode setting succeeds, re-enable motor to operation state
    if (current_mode == mode)
    {
        RCLCPP_INFO(this->get_logger(), "✅ Operation mode set successfully, re-enabling motor");
        
        // State machine conversion: Shutdown -> Switch on -> Enable operation
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Final confirmation
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "👉🏻 Final status word: 0x%04X", status_word);
        RCLCPP_INFO(this->get_logger(), "✅ Operation mode switch successful: %d", mode);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Operation mode switch failed, current mode: %d, expected mode: %d", 
                    current_mode, mode);
        RCLCPP_ERROR(this->get_logger(), "⚠️ Please check if motor supports mode %d, or refer to motor documentation", mode);
    }
}

void CANopenROS2::set_profile_velocity(float velocity_deg_per_sec)
{
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    write_sdo(OD_PROFILE_VELOCITY, 0x00, velocity_units, 4);
    RCLCPP_INFO(this->get_logger(), "🏃 Profile velocity set: %.2f°/s", velocity_deg_per_sec);
}

void CANopenROS2::set_profile_acceleration(float acceleration_deg_per_sec2)
{
    int32_t acceleration_units = acceleration_to_units(acceleration_deg_per_sec2);
    write_sdo(OD_PROFILE_ACCELERATION, 0x00, acceleration_units, 4);
    RCLCPP_INFO(this->get_logger(), "⬆️ Profile acceleration set: %.2f°/s²", acceleration_deg_per_sec2);
}

void CANopenROS2::set_profile_deceleration(float deceleration_deg_per_sec2)
{
    int32_t deceleration_units = acceleration_to_units(deceleration_deg_per_sec2);
    write_sdo(OD_PROFILE_DECELERATION, 0x00, deceleration_units, 4);
    RCLCPP_INFO(this->get_logger(), "⬇️ Profile deceleration set: %.2f°/s²", deceleration_deg_per_sec2);
}

void CANopenROS2::set_profile_parameters(float velocity_deg_per_sec, float acceleration_deg_per_sec2, float deceleration_deg_per_sec2)
{
    set_profile_velocity(velocity_deg_per_sec);
    set_profile_acceleration(acceleration_deg_per_sec2);
    set_profile_deceleration(deceleration_deg_per_sec2);
    RCLCPP_INFO(this->get_logger(), "📏 Profile parameters set - velocity: %.2f°/s, acceleration: %.2f°/s², deceleration: %.2f°/s²", 
               velocity_deg_per_sec, acceleration_deg_per_sec2, deceleration_deg_per_sec2);
}

void CANopenROS2::set_control_word(uint16_t control_word)
{
    // Send control word using PDO
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 2;
    frame.data[0] = control_word & 0xFF;  // Control word low byte
    frame.data[1] = (control_word >> 8) & 0xFF;  // Control word high byte
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to send control word");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "🎮 Control word sent: 0x%04X", control_word);
    }
}

void CANopenROS2::set_target_velocity(int32_t velocity_units_per_sec)
{
    // DSY-C.EDS specifies target velocity at 0x60FF
    write_sdo(OD_TARGET_VELOCITY, 0x00, velocity_units_per_sec, 4);
    RCLCPP_INFO(this->get_logger(), "🎯 Target velocity set (command units): %d", velocity_units_per_sec);
}

void CANopenROS2::go_to_position(float angle)
{
    RCLCPP_INFO(this->get_logger(), "📍 Move to position: %.2f°", angle);
    
    int32_t position = angle_to_position(angle);
    RCLCPP_INFO(this->get_logger(), "🎯 Target position command units: %d", position);
    
    // First use SDO to set target position (one-time setting)
    write_sdo(OD_TARGET_POSITION, 0x00, position, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Use PDO for control word handshake to avoid state machine confusion from mixing with SDO
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 6;  // 2 bytes control, 4 bytes position
    
    // 1. Send Enable Operation (0x000F) + target position
    // Ensure position data is always in PDO to prevent data inconsistency
    frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
    frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;
    frame.data[2] = position & 0xFF;
    frame.data[3] = (position >> 8) & 0xFF;
    frame.data[4] = (position >> 16) & 0xFF;
    frame.data[5] = (position >> 24) & 0xFF;
    
    ssize_t write_result = write(can_socket_, &frame, sizeof(struct can_frame));
    (void)write_result;  // Explicitly ignore return value
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Brief wait
    
    // 2. Send Enable Operation + New Setpoint (0x001F) + target position (rising edge trigger)
    frame.data[0] = (CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT) & 0xFF;
    write_result = write(can_socket_, &frame, sizeof(struct can_frame));
    (void)write_result;  // Explicitly ignore return value
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // 3. Restore Enable Operation (0x000F) + target position (complete handshake)
    frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
    write_result = write(can_socket_, &frame, sizeof(struct can_frame));
    (void)write_result;  // Explicitly ignore return value
    send_sync_frame();
    
    RCLCPP_INFO(this->get_logger(), "📤 Position command sent via PDO handshake");
    
    // If target position is current position, no need to wait
    int32_t current_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
    int32_t position_diff = (position > current_pos) ? (position - current_pos) : (current_pos - position);
    if (position_diff < 100)  // If position difference is less than 100 command units, consider arrived
    {
        RCLCPP_INFO(this->get_logger(), "✅ Target position close to current position, no movement needed");
        return;
    }
    
    // Monitor if target position is reached
    int retry = 0;
    const int max_retries = 50;  // Increased to 50 times, total 10 seconds
    while (retry < max_retries)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Read status word
        int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
        
        // Check if fault occurred
        if (status_word & 0x0008 || (status_word & 0x004F) == 0x0040)
        {
             RCLCPP_ERROR(this->get_logger(), "🚨 Fault or inhibit state detected during movement (0x%04X)", status_word);
             check_and_clear_error(); // Diagnose error
             // Try to re-enable? Or exit directly
             break;
        }

        // Read actual position
        int32_t actual_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
        int32_t current_diff = (position > actual_pos) ? (position - actual_pos) : (actual_pos - position);
        
        // Check target reached bit (bit 10) or position error is small enough
        if ((status_word & 0x0400) || current_diff < 100)
        {
            RCLCPP_INFO(this->get_logger(), "🎯 Target position reached (status word: 0x%04X, position difference: %d)", status_word, current_diff);
            break;
        }
        
        // Output progress every 5 retries
        if (retry % 5 == 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "⏳ Waiting for target to arrive... (retry %d/%d, position difference: %d)", retry, max_retries, current_diff);
        }
        
        retry++;
    }
    
    if (retry >= max_retries)
    {
        int32_t final_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
        int32_t final_diff = (position > final_pos) ? (position - final_pos) : (final_pos - position);
        RCLCPP_WARN(this->get_logger(), "⏰ Timeout waiting for target position to arrive (final position difference: %d command units, approximately %.2f°)", 
                   final_diff, position_to_angle(final_diff));
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
    
    // Set profile velocity parameter (for velocity mode)
    set_profile_velocity(velocity_deg_per_sec);
    
    // Convert to command units
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    
    // Ensure motor is in operation enabled state
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    uint16_t control_word = CONTROL_ENABLE_OPERATION;
    
    if ((status_word & 0x006F) != 0x0027)  // If not in "operation enabled" state
    {
        RCLCPP_INFO(this->get_logger(), "⚡ Motor not in operation state, enabling...");
        // State machine conversion
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Send control word and target velocity using RxPDO2 (real-time control)
    struct can_frame frame;
    frame.can_id = COB_RPDO2 + node_id_;
    frame.can_dlc = 6;  // Control word (2 bytes) + target velocity (4 bytes)
    
    // Control word (0x6040) - 2 bytes
    frame.data[0] = control_word & 0xFF;  // Control word low byte
    frame.data[1] = (control_word >> 8) & 0xFF;  // Control word high byte
    
    // Target velocity (0x60FF) - 4 bytes (little-endian)
    frame.data[2] = velocity_units & 0xFF;
    frame.data[3] = (velocity_units >> 8) & 0xFF;
    frame.data[4] = (velocity_units >> 16) & 0xFF;
    frame.data[5] = (velocity_units >> 24) & 0xFF;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to send velocity PDO [node %d]", node_id_);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "📤 Velocity PDO sent [node %d]: %.2f°/s (command units: %d)", 
                    node_id_, velocity_deg_per_sec, velocity_units);
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
 *          Unit on the drive: position command units / s²  (same scaling as profile acceleration).
 * @param deceleration_rev_per_sec2  Quick stop deceleration [r/s²]
 */
void CANopenROS2::set_quick_stop_deceleration(float deceleration_rev_per_sec2)
{
    // Convert r/s² to position command units/s²:
    //   1 revolution = position_factor_numerator_ / position_factor_denominator_ command units
    //   command_units/s² = deceleration_rev_per_sec2 * (num / den)
    uint32_t decel_units = static_cast<uint32_t>(std::lround(
        static_cast<double>(deceleration_rev_per_sec2) * (static_cast<double>(position_factor_numerator_) / static_cast<double>(position_factor_denominator_))));

    write_sdo(OD_QUICK_STOP_DECEL, 0x00, static_cast<int32_t>(decel_units), 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RCLCPP_INFO(this->get_logger(),
        "🛑 Quick stop deceleration set: %.2f r/s² (%u command units/s²) (0x6085)",
        deceleration_rev_per_sec2, decel_units);
}

/**
 * @brief 📌 Set maximum profile velocity (0x607F)
 * @details Writes the upper velocity limit to the drive. The drive will clamp
 *          any profile velocity (0x6081) command to this value.
 *          Unit accepted here: r/s (revolutions per second of the output shaft).
 *          Conversion: command_units/s = rev_per_sec × (position_factor_numerator_ / position_factor_denominator_)
 * @param velocity_rev_per_sec  Max profile velocity [r/s]
 */
void CANopenROS2::set_max_profile_velocity(float velocity_rev_per_sec)
{
    // Convert r/s → command units/s
    //   1 revolution = position_factor_numerator_ / position_factor_denominator_ command units
    uint32_t velocity_units = static_cast<uint32_t>(std::lround(
        static_cast<double>(velocity_rev_per_sec) * (static_cast<double>(position_factor_numerator_) / static_cast<double>(position_factor_denominator_))));

    write_sdo(OD_MAX_PROFILE_VELOCITY, 0x00, static_cast<int32_t>(velocity_units), 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RCLCPP_INFO(this->get_logger(),
        "📌 Max profile velocity set: %.2f r/s (%u command units/s) (0x607F)",
        velocity_rev_per_sec, velocity_units);
}

/**
 * @brief ⚖️ Set position factor (0x6093)
 * @details Configures the drive's internal position scaling between the host-side
 *          position demand value (what we write to 0x607A / read from 0x6064) and
 *          the encoder-increment domain used by the drive hardware.
 *
 *          CiA 402 formula:
 *            position_demand [drive units] = encoder_increments × Numerator / Denominator
 *
 *          Typical use: set Numerator = desired_units_per_revolution and
 *                           Denominator = encoder_counts_per_revolution
 *          so that 1 position unit on the host equals exactly 1 encoder count × scaling.
 *
 *          With Numerator = Denominator = 1 (unity, the default) the host unit equals
 *          one raw encoder increment and all unit conversion remains in software.
 *
 * @param numerator    0x6093:01  (uint32)
 * @param denominator  0x6093:02  (uint32, must be ≠ 0)
 */
void CANopenROS2::set_position_factor(uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0)
    {
        RCLCPP_ERROR(this->get_logger(),
            "❌ position_factor_denominator must not be 0 – skipping 0x6093 write");
        return;
    }

    // Sub-index 1: Numerator
    write_sdo(OD_POSITION_FACTOR, 0x01, static_cast<int32_t>(numerator), 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Sub-index 2: Denominator
    write_sdo(OD_POSITION_FACTOR, 0x02, static_cast<int32_t>(denominator), 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RCLCPP_INFO(this->get_logger(),
        "⚖️ Position factor set: %u / %u (0x6093:01 / 0x6093:02)",
        numerator, denominator);
}
