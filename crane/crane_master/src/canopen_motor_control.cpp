#include "crane_master/canopen_ros2_node.hpp"
#include <cmath>

void CANopenROS2::initialize_node()
{
    // Send NMT stop command
    send_nmt_command(NMT_STOP_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Send NMT reset command
    send_nmt_command(NMT_RESET_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // First enable the motor, then set the operation mode
    // Shutdown
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Switch on
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Enable operation
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Read status word to confirm motor is enabled
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "Status word after enable: 0x%04X", status_word);
    
    // Now try to set operation mode (default is position mode)
    write_sdo(OD_OPERATION_MODE, 0x00, MODE_PROFILE_POSITION, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Verify operation mode
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "Current operation mode: %d", mode);
    
    // If operation mode is still not position mode, try setting via PDO
    if (mode != MODE_PROFILE_POSITION)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set operation mode using SDO, trying PDO");
        
        // Set operation mode via PDO
        struct can_frame frame;
        frame.can_id = COB_RPDO1 + node_id_;
        frame.can_dlc = 3;  // control word (2 bytes) + operation mode (1 byte)
        frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;  // control word low byte
        frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;  // control word high byte
        frame.data[2] = MODE_PROFILE_POSITION;  // operation mode
        
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
        
        // Verify operation mode again
        mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "Operation mode after PDO setting: %d", mode);
    }
    
    // Set profile velocity
    set_profile_velocity(profile_velocity_);
    
    // Set profile acceleration
    set_profile_acceleration(profile_acceleration_);
    
    // Set profile deceleration
    set_profile_deceleration(profile_deceleration_);
    
    // Disable sync generator
    write_sdo(OD_CYCLE_PERIOD, 0x00, 0, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Set communication cycle
    write_sdo(OD_CYCLE_PERIOD, 0x00, cycle_period_us_, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "Node initialization completed");
}

void CANopenROS2::configure_pdo()
{
    RCLCPP_INFO(this->get_logger(), "Configuring PDO mapping...");
    
    // Enter pre-operational state
    send_nmt_command(NMT_STOP_REMOTE_NODE);
    RCLCPP_INFO(this->get_logger(), "Entered pre-operational state");
    
    // Configure TxPDO1
    RCLCPP_INFO(this->get_logger(), "Starting to configure TxPDO1");
    
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
    
    // 4. Map object: status word
    write_sdo(0x1A00, 0x01, 0x60410010, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 5. Map object: actual position
    write_sdo(0x1A00, 0x02, 0x60640020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 6. Set number of TxPDO1 mapped objects to 2
    write_sdo(0x1A00, 0x00, 0x02, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 7. Set transmission type (synchronous) and enable TxPDO1
    write_sdo(0x1800, 0x02, 0x01, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_sdo(0x1800, 0x01, txpdo1_cob_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure RxPDO1
    RCLCPP_INFO(this->get_logger(), "Starting to configure RxPDO1");
    
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
    
    // 4. Map object: control word
    write_sdo(0x1600, 0x01, 0x60400010, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 5. Map object: target position
    write_sdo(0x1600, 0x02, 0x607A0020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 6. Set number of RxPDO1 mapped objects to 2
    write_sdo(0x1600, 0x00, 0x02, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 7. Set transmission type (synchronous) and enable RxPDO1
    write_sdo(0x1400, 0x02, 0x01, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_sdo(0x1400, 0x01, rxpdo1_cob_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure RxPDO2 for velocity control
    RCLCPP_INFO(this->get_logger(), "🫡 Configuring RxPDO2 for velocity control");
    
    // 1. Disable RxPDO2 (COB-ID 0x300 + NodeID)
    uint32_t rxpdo2_cob_id = 0x300 + node_id_;
    write_sdo(0x1401, 0x01, rxpdo2_cob_id | 0x80000000, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. Set transmission type
    // Keep RPDO2 asynchronous so velocity commands work without a continuous SYNC producer.
    write_sdo(0x1401, 0x02, 0xFF, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 3. Clear mapping
    write_sdo(0x1601, 0x00, 0x00, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 4. Map control word (0x6040)
    write_sdo(0x1601, 0x01, 0x60400010, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 5. Map target velocity (0x60FF) <-- This enables real-time velocity control
    write_sdo(0x1601, 0x02, 0x60FF0020, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 6. Set number of entries to 2
    write_sdo(0x1601, 0x00, 0x02, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 7. Keep RxPDO2 asynchronous and enable it.
    write_sdo(0x1401, 0x02, 0xFF, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_sdo(0x1401, 0x01, rxpdo2_cob_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "👍👍👍 PDO configuration completed");
}

void CANopenROS2::start_node()
{
    RCLCPP_INFO(this->get_logger(), "🤩 Starting node...");
    
    // Send NMT start command
    send_nmt_command(NMT_START_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Get actual position
    int32_t actual_position = read_sdo(OD_ACTUAL_POSITION, 0x00);
    float actual_angle = position_to_angle(actual_position);
    RCLCPP_INFO(this->get_logger(), "Actual position: %.2f°", actual_angle);
    
    // Send sync frame
    send_sync_frame();
    
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Node startup completed");
}

void CANopenROS2::set_immediate_effect(bool immediate)
{
    RCLCPP_INFO(this->get_logger(), "👀 Setting %s effect", immediate ? "immediate" : "non-immediate");
    
    // Read current control word
    int32_t controlword = read_sdo(OD_CONTROL_WORD, 0x00);
    
    if (immediate)
    {
        controlword |= (1 << 5);  // set bit 5 to 1 (immediate effect)
    }
    else
    {
        controlword &= ~(1 << 5);  // set bit 5 to 0 (non-immediate effect)
    }
    
    // Write new control word
    write_sdo(OD_CONTROL_WORD, 0x00, controlword, 2);
    
    RCLCPP_INFO(this->get_logger(), "🤖 Control word updated to: 0x%04X", controlword);
}

void CANopenROS2::check_and_clear_error()
{
    // Check Error Register (0x1001)
    int32_t error_register = read_sdo(0x1001, 0x00);
    if (error_register > 0)
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Error Register (0x1001): 0x%02X", error_register);
        
        // Read Manufacturer Error Code (0x603F) if supported
        int32_t manufacturer_error = read_sdo(0x603F, 0x00);
        if (manufacturer_error > 0) {
             RCLCPP_ERROR(this->get_logger(), "😩 Manufacturer Error (0x603F): 0x%04X", manufacturer_error);
        }
    }
}

void CANopenROS2::clear_fault()
{
    RCLCPP_INFO(this->get_logger(), "🫥 Clearing fault...");
    
    check_and_clear_error();

    // Send fault reset command
    set_control_word(CONTROL_FAULT_RESET);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Enter ready state
    set_control_word(CONTROL_SHUTDOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "🫥 Fault cleared");
}

void CANopenROS2::enable_motor()
{
    RCLCPP_INFO(this->get_logger(), "🫥 Enabling motor...");
    
    // Read current status word
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "🤖 Current status word: 0x%04X", status_word);
    
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
        RCLCPP_WARN(this->get_logger(), "😅 Unable to read status word, attempting to continue enable process");
    }
    else
    {
        status_word_ = static_cast<uint16_t>(status_word);
    }
    
    // Check if there is a fault that needs clearing
    if (status_word_ & 0x0008)  // fault bit
    {
        RCLCPP_WARN(this->get_logger(), "😅 Fault detected, attempting to clear...");
        clear_fault();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else if ((status_word_ & 0x004F) == 0x0040) // Operation Inhibit
    {
        RCLCPP_WARN(this->get_logger(), "😅 Operation Inhibit (0x0040) detected, attempting reset...");
        // Strict reset sequence: 0x0080 -> 0x0006 -> 0x0007 -> 0x000F
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
    
    // Execute state transition sequence - strict sequence for robustness
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
        RCLCPP_INFO(this->get_logger(), "🤖 Status word after enable: 0x%04X", status_word_);
        
        // Check if enable was successful
        if ((status_word_ & 0x006F) == 0x0027)
        {
            RCLCPP_INFO(this->get_logger(), "👍👍👍 Motor successfully enabled (operation enabled)");
        }
        else if ((status_word_ & 0x006F) == 0x0023)
        {
            RCLCPP_INFO(this->get_logger(), "🤖 Motor switched on but not in operation mode");
        }
        else if ((status_word_ & 0x004F) == 0x0040)
        {
            RCLCPP_WARN(this->get_logger(), "😅 Motor still in operation inhibit state, may need to check hardware or configuration");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "😅 Unable to read status word after enable");
    }
    
    // Try enabling again using PDO (more robust, periodic refresh)
    set_control_word(CONTROL_SHUTDOWN);  // shutdown
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    set_control_word(CONTROL_SWITCH_ON);  // switch on
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    set_control_word(CONTROL_ENABLE_OPERATION);  // enable operation
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Motor enabled");
}

void CANopenROS2::stop_motor()
{
    RCLCPP_INFO(this->get_logger(), "🫥 Stopping motor...");
    
    // Set target velocity to 0
    set_target_velocity(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Disable operation
    set_control_word(CONTROL_SWITCH_ON);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Shutdown motor
    set_control_word(CONTROL_SHUTDOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Disable voltage
    set_control_word(CONTROL_DISABLE_VOLTAGE);
    
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Motor stopped");
}

void CANopenROS2::initialize_motor()
{
    // Initialize node
    initialize_node();
    
    // Configure PDO mapping
    configure_pdo();
    
    // Wait for a short period
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Start node
    start_node();
    
    // Set immediate effect
    set_immediate_effect(true);
    
    // Clear faults
    clear_fault();
    
    // Enable motor
    enable_motor();
    
    // Read and log gear ratio (0x6091:01 and 0x6091:02) - read only for reference, do not modify
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
        RCLCPP_INFO(this->get_logger(), "😅 Unable to read or invalid gear ratio (0x6091), using configured value: %.2f", gear_ratio_);
    }
    
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Motor initialization completed");
}

void CANopenROS2::set_operation_mode(uint8_t mode)
{
    RCLCPP_INFO(this->get_logger(), "👀 Starting to switch operation mode to: %d", mode);
    
    // Step 1: read current status word and mode
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    int32_t current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "🤖 Current status word: 0x%04X, current mode: %d", status_word, current_mode);
    
    // If already in target mode, return directly
    if (current_mode == mode)
    {
        RCLCPP_INFO(this->get_logger(), "👍👍👍 Already in target mode: %d", mode);
        return;
    }
    
    // Step 2: ensure motor is in "Ready to switch on" (0x0021) or "Switched on" (0x0023)
    if ((status_word & 0x006F) != 0x0021 && (status_word & 0x006F) != 0x0023)
    {
        RCLCPP_INFO(this->get_logger(), "🤖 Motor not in state allowing mode switch, transitioning state...");
        
        // First disable operation
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Transition to "Ready to switch on"
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Check status
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "🤖 Status word after state transition: 0x%04X", status_word);
    }
    
    // Step 3: set operation mode (must be in "Ready to switch on" or "Switched on" state)
    RCLCPP_INFO(this->get_logger(), "👀 Setting operation mode to: %d", mode);
    write_sdo(OD_OPERATION_MODE, 0x00, mode, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // Check whether operation mode was set successfully
    current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "🤖 Operation mode after setting: %d", current_mode);
    
    // If SDO setting failed, try using PDO
    if (current_mode != mode)
    {
        RCLCPP_WARN(this->get_logger(), "😅 Failed to set mode using SDO, trying PDO");
        
        // Send operation mode via PDO (through RPDO1 if mapped)
        // Note: this requires RPDO1 mapping to include operation mode object
        struct can_frame frame;
        frame.can_id = COB_RPDO1 + node_id_;
        frame.can_dlc = 3;  // control word (2 bytes) + operation mode (1 byte)
        frame.data[0] = CONTROL_SWITCH_ON & 0xFF;  // control word low byte
        frame.data[1] = (CONTROL_SWITCH_ON >> 8) & 0xFF;  // control word high byte
        frame.data[2] = mode;  // operation mode
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "😩 Failed to send PDO operation mode");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "⏩⏩⏩ PDO operation mode sent: %d", mode);
        }
        
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Check operation mode again
        current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "🤖 Operation mode after PDO setting: %d", current_mode);
    }
    
    // Step 4: if mode was set successfully, re-enable motor to operation state
    if (current_mode == mode)
    {
        RCLCPP_INFO(this->get_logger(), "👍👍👍 Operation mode set successfully, re-enabling motor");
        
        // State machine transition: Shutdown -> Switch on -> Enable operation
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Final confirmation
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "🤖 Final status word: 0x%04X", status_word);
        RCLCPP_INFO(this->get_logger(), "Operation mode switch successful: %d", mode);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Operation mode switch failed, current mode: %d, expected mode: %d", 
                    current_mode, mode);
        RCLCPP_ERROR(this->get_logger(), "😩 Please check if motor supports mode %d, or refer to motor documentation", mode);
    }
}

void CANopenROS2::set_profile_velocity(float velocity_deg_per_sec)
{
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    write_sdo(OD_PROFILE_VELOCITY, 0x00, velocity_units, 4);
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Profile velocity set: %.2f°/s", velocity_deg_per_sec);
}

void CANopenROS2::set_profile_acceleration(float acceleration_deg_per_sec2)
{
    int32_t acceleration_units = acceleration_to_units(acceleration_deg_per_sec2);
    write_sdo(OD_PROFILE_ACCELERATION, 0x00, acceleration_units, 4);
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Profile acceleration set: %.2f°/s²", acceleration_deg_per_sec2);
}

void CANopenROS2::set_profile_deceleration(float deceleration_deg_per_sec2)
{
    int32_t deceleration_units = acceleration_to_units(deceleration_deg_per_sec2);
    write_sdo(OD_PROFILE_DECELERATION, 0x00, deceleration_units, 4);
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Profile deceleration set: %.2f°/s²", deceleration_deg_per_sec2);
}

void CANopenROS2::set_profile_parameters(float velocity_deg_per_sec, float acceleration_deg_per_sec2, float deceleration_deg_per_sec2)
{
    set_profile_velocity(velocity_deg_per_sec);
    set_profile_acceleration(acceleration_deg_per_sec2);
    set_profile_deceleration(deceleration_deg_per_sec2);
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Profile parameters set - velocity: %.2f°/s, acceleration: %.2f°/s², deceleration: %.2f°/s²", 
               velocity_deg_per_sec, acceleration_deg_per_sec2, deceleration_deg_per_sec2);
}

void CANopenROS2::set_control_word(uint16_t control_word)
{
    // Send control word via PDO
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 2;
    frame.data[0] = control_word & 0xFF;  // control word low byte
    frame.data[1] = (control_word >> 8) & 0xFF;  // control word high byte
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to send control word");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "🤖 Control word sent: 0x%04X", control_word);
    }
}

void CANopenROS2::set_target_velocity(int32_t velocity_units_per_sec)
{
    // DSY-C.EDS specifies target velocity at 0x60FF
    write_sdo(OD_TARGET_VELOCITY, 0x00, velocity_units_per_sec, 4);
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Target velocity set (command units): %d", velocity_units_per_sec);
}

void CANopenROS2::ensure_operation_enabled()
{
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    if ((status_word & 0x006F) == 0x0027)
    {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Motor is not operation enabled (status=0x%04X), enabling before homing", status_word);
    enable_motor();
}

int32_t CANopenROS2::configure_homing_parameters()
{
    const int32_t homing_fast_units = velocity_to_units(homing_fast_velocity_);
    const int32_t homing_slow_units = velocity_to_units(homing_slow_velocity_);
    const int32_t homing_accel_units = acceleration_to_units(homing_acceleration_);

    write_sdo(OD_HOMING_METHOD, 0x00, homing_method_, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write_sdo(OD_HOMING_SPEED, 0x01, homing_fast_units, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write_sdo(OD_HOMING_SPEED, 0x02, homing_slow_units, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write_sdo(OD_HOMING_ACCELERATION, 0x00, homing_accel_units, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const int32_t method_feedback = read_sdo(OD_HOMING_METHOD, 0x00);

    RCLCPP_INFO(
        this->get_logger(),
        "Homing parameters configured: method=%d (feedback=%d), 0x6099:01=%d, 0x6099:02=%d, 0x609A=%d",
        homing_method_,
        method_feedback,
        homing_fast_units,
        homing_slow_units,
        homing_accel_units);

    return method_feedback;
}

void CANopenROS2::store_parameters()
{
    // 0x1010:01 is optional in the CANopen standard.  Not all drives implement it.
    // write_sdo is fire-and-forget; a drive that rejects the write will respond with
    // an SDO abort (0x06020000 = "object does not exist") logged by receive_can_frames.
    // This function is intentionally best-effort: a failure here does not abort homing.
    RCLCPP_INFO(this->get_logger(), "Requesting parameter store through 0x1010:01 (best-effort)");
    write_sdo(OD_STORE_PARAMETERS, 0x01, STORE_SIGNATURE, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

void CANopenROS2::restore_mode_after_homing(int32_t previous_mode, int32_t actual_position)
{
    if (!homing_restore_previous_mode_ ||
        previous_mode == MODE_HOMING ||
        (previous_mode != MODE_PROFILE_POSITION && previous_mode != MODE_PROFILE_VELOCITY))
    {
        return;
    }

    set_operation_mode(static_cast<uint8_t>(previous_mode));
    const int32_t restored_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    if (restored_mode == MODE_PROFILE_POSITION)
    {
        write_sdo(OD_TARGET_POSITION, 0x00, actual_position, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    else if (restored_mode == MODE_PROFILE_VELOCITY)
    {
        write_sdo(OD_TARGET_VELOCITY, 0x00, 0, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

bool CANopenROS2::set_current_position_as_home(std::string & result_message)
{
    const int32_t actual_before = read_sdo(OD_ACTUAL_POSITION, 0x00);
    const int32_t home_offset_before = read_sdo(OD_HOME_OFFSET, 0x00);
    const int32_t zero_tolerance = 100;

    int32_t candidate_offsets[2] = {
        home_offset_before - actual_before,
        home_offset_before + actual_before,
    };

    int32_t best_offset = home_offset_before;
    int32_t best_actual = actual_before;
    int best_candidate = -1;
    int last_written_candidate = -1;

    for (int i = 0; i < 2; ++i)
    {
        write_sdo(OD_HOME_OFFSET, 0x00, candidate_offsets[i], 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const int32_t actual_after = read_sdo(OD_ACTUAL_POSITION, 0x00);
        RCLCPP_INFO(
            this->get_logger(),
            "Method 35 candidate %d: 0x607C=%d, actual position readback=%d",
            i + 1,
            candidate_offsets[i],
            actual_after);

        if (std::abs(actual_after) < std::abs(best_actual))
        {
            best_offset = candidate_offsets[i];
            best_actual = actual_after;
            best_candidate = i;
        }

        last_written_candidate = i;
        if (std::abs(actual_after) <= zero_tolerance)
        {
            break;
        }
    }

    if (best_candidate < 0)
    {
        result_message =
            "Method 35 could not move the logical position near zero through 0x607C";
        return false;
    }

    if (best_candidate != last_written_candidate)
    {
        write_sdo(OD_HOME_OFFSET, 0x00, best_offset, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        best_actual = read_sdo(OD_ACTUAL_POSITION, 0x00);
    }

    if (std::abs(best_actual) > zero_tolerance)
    {
        result_message =
            "Method 35 wrote 0x607C but actual position is still not close to zero (" +
            std::to_string(best_actual) + ")";
        return false;
    }

    if (homing_store_parameters_)
    {
        store_parameters();
    }

    position_ = best_actual;
    homing_completed_ = true;
    home_position_ = 0;

    result_message = homing_store_parameters_
        ? "Current position saved as home using 0x607C; 0x1010 store requested (drive may not persist it across power cycles)"
        : "Current position saved as home using 0x607C (session only, not persisted to EEPROM)";
    return true;
}

// Send the CiA402 homing start sequence (bit-4 rising edge via PDO+SYNC).
// Called from execute_homing after mode 6 and homing parameters are configured.
static void send_homing_start(int can_socket, uint8_t node_id, int32_t current_pos)
{
    struct can_frame hm_frame;
    hm_frame.can_id = COB_RPDO1 + node_id;
    hm_frame.can_dlc = 6;
    hm_frame.data[2] = current_pos & 0xFF;
    hm_frame.data[3] = (current_pos >> 8) & 0xFF;
    hm_frame.data[4] = (current_pos >> 16) & 0xFF;
    hm_frame.data[5] = (current_pos >> 24) & 0xFF;

    // 1. Ensure bit 4 is low
    hm_frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
    hm_frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;
    write(can_socket, &hm_frame, sizeof(struct can_frame));

    // 2. Rising edge on bit 4 (0x1F) triggers homing
    uint16_t hm_start = CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT;
    hm_frame.data[0] = hm_start & 0xFF;
    hm_frame.data[1] = (hm_start >> 8) & 0xFF;
    write(can_socket, &hm_frame, sizeof(struct can_frame));

    // 3. Clear bit 4
    hm_frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
    hm_frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;
    write(can_socket, &hm_frame, sizeof(struct can_frame));
}

bool CANopenROS2::execute_homing(std::string & result_message)
{
    const int32_t previous_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    homing_completed_ = false;
    home_position_ = 0;

    ensure_operation_enabled();
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    if ((status_word & 0x006F) != 0x0027)
    {
        result_message = "Servo is not in Operation Enabled state, homing aborted";
        return false;
    }

    // Always try the drive's native homing mode (CiA402 mode 6) first, including
    // for method 35.  For method 35 the drive should complete near-instantly (no
    // physical movement) so a short 3-second timeout is used.  If the drive does
    // not support native method 35 the code falls back to the manual 0x607C trick.
    set_operation_mode(MODE_HOMING);
    const int32_t mode_display = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);

    if (mode_display != MODE_HOMING)
    {
        if (homing_method_ == 35)
        {
            RCLCPP_WARN(this->get_logger(),
                "Failed to enter homing mode (0x6061=%d); falling back to manual 0x607C for method 35",
                mode_display);
            const bool success = set_current_position_as_home(result_message);
            if (success)
            {
                restore_mode_after_homing(previous_mode, position_);
            }
            return success;
        }
        result_message =
            "Failed to enter HM: 0x6060 was set to 6 but 0x6061 reports " + std::to_string(mode_display);
        return false;
    }

    const int32_t method_feedback = configure_homing_parameters();

    // If the drive rejected the homing method write (0x6098 reads back a different
    // value), there is no point waiting – the native homing engine will never start.
    // For method 35 this is a known DSY-C limitation: immediately fall back to the
    // manual 0x607C approach.
    if (method_feedback != homing_method_)
    {
        if (homing_method_ == 35)
        {
            RCLCPP_WARN(this->get_logger(),
                "Drive rejected homing method 35 (0x6098 reads back %d); "
                "skipping native attempt, using manual 0x607C approach",
                method_feedback);
            set_operation_mode(MODE_PROFILE_POSITION);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            const bool success = set_current_position_as_home(result_message);
            if (success)
            {
                restore_mode_after_homing(previous_mode, position_);
            }
            return success;
        }
        result_message =
            "Drive rejected homing method " + std::to_string(homing_method_) +
            " (0x6098 reads back " + std::to_string(method_feedback) + ")";
        return false;
    }

    // PDO + SYNC rising-edge trigger (CiA402 requirement)
    const int32_t current_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_homing_start(can_socket_, node_id_, current_pos);
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_sync_frame();

    // Method 35 should complete without movement – 3 s is plenty if we reach here.
    // All other methods use the configured timeout.
    const double effective_timeout =
        (homing_method_ == 35) ? 3.0 : homing_timeout_s_;

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int64_t>(effective_timeout * 1000.0));
    int log_counter = 0;

    while (std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        status_word_ = static_cast<uint16_t>(status_word);

        if (status_word & 0x0008)
        {
            check_and_clear_error();
            result_message = "Homing stopped because the drive entered fault state";
            return false;
        }

        // Bit 13 = homing error.  For method 35, the drive may set this to indicate
        // it does not support the method natively → fall back to manual 0x607C.
        if (status_word & 0x2000)
        {
            check_and_clear_error();
            if (homing_method_ == 35)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Drive reported homing error for native method 35 (bit 13); "
                    "falling back to manual 0x607C approach");
                set_operation_mode(MODE_PROFILE_POSITION);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                const bool success = set_current_position_as_home(result_message);
                if (success)
                {
                    restore_mode_after_homing(previous_mode, position_);
                }
                return success;
            }
            result_message = "Homing failed: statusword bit 13 indicates homing error";
            return false;
        }

        // Bit 12 = homing completed successfully
        if (status_word & 0x1000)
        {
            const int32_t actual_position = read_sdo(OD_ACTUAL_POSITION, 0x00);
            position_ = actual_position;
            homing_completed_ = true;
            home_position_ = actual_position;

            if (homing_store_parameters_)
            {
                store_parameters();
            }

            restore_mode_after_homing(previous_mode, actual_position);

            result_message = homing_store_parameters_
                ? "Homing completed: statusword bit 12 set, bit 13 clear, parameters stored"
                : "Homing completed: statusword bit 12 set, bit 13 clear";
            return true;
        }

        if ((log_counter++ % 10) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for homing result, statusword=0x%04X", status_word);
        }
    }

    // Timeout – for method 35 try the manual 0x607C fallback before giving up
    if (homing_method_ == 35)
    {
        RCLCPP_WARN(this->get_logger(),
            "Native homing timed out for method 35 (drive may not support it); "
            "falling back to manual 0x607C approach");
        set_operation_mode(MODE_PROFILE_POSITION);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const bool success = set_current_position_as_home(result_message);
        if (success)
        {
            restore_mode_after_homing(previous_mode, position_);
        }
        return success;
    }

    result_message = "Homing timeout waiting for statusword bit 12";
    return false;
}

void CANopenROS2::go_to_position(float angle)
{
    RCLCPP_INFO(this->get_logger(), "👀 Moving to position: %.2f°", angle);
    
    int32_t position = angle_to_position(angle);
    RCLCPP_INFO(this->get_logger(), "🤖 Target position command units: %d", position);
    
    // First set target position via SDO (one-shot)
    write_sdo(OD_TARGET_POSITION, 0x00, position, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Use PDO for control word handshake to avoid confusing the state machine with mixed SDO/PDO
    struct can_frame frame;
    frame.can_id = COB_RPDO1 + node_id_;
    frame.can_dlc = 6;  // 2 bytes control, 4 bytes position
    
    // 1. Send Enable Operation (0x000F) + target position
    // Ensure position data is always in PDO to prevent inconsistencies
    frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
    frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;
    frame.data[2] = position & 0xFF;
    frame.data[3] = (position >> 8) & 0xFF;
    frame.data[4] = (position >> 16) & 0xFF;
    frame.data[5] = (position >> 24) & 0xFF;
    
    write(can_socket_, &frame, sizeof(struct can_frame));
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // short wait
    
    // 2. Send Enable Operation + New Setpoint (0x001F) + target position (rising-edge trigger)
    frame.data[0] = (CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT) & 0xFF;
    write(can_socket_, &frame, sizeof(struct can_frame));
    send_sync_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // 3. Restore Enable Operation (0x000F) + target position (complete handshake)
    frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;
    write(can_socket_, &frame, sizeof(struct can_frame));
    send_sync_frame();
    
    RCLCPP_INFO(this->get_logger(), "👍👍👍 Position command sent via PDO handshake");
    
    // If target position is equal to current position, no need to wait
    int32_t current_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
    int32_t position_diff = (position > current_pos) ? (position - current_pos) : (current_pos - position);
    if (position_diff < 100)  // if diff < 100 command units, treat as reached
    {
        RCLCPP_INFO(this->get_logger(), "👍👍👍 Target position close to current position, no movement needed");
        return;
    }
    
    // Monitor whether target position is reached
    int retry = 0;
    const int max_retries = 50;  // 50 * 200ms = 10 seconds
    while (retry < max_retries)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Read status word
        int32_t status_word_read = read_sdo(OD_STATUS_WORD, 0x00);
        
        // Check for fault or inhibit
        if (status_word_read & 0x0008 || (status_word_read & 0x004F) == 0x0040)
        {
             RCLCPP_ERROR(this->get_logger(), "😩 Fault or operation inhibit state detected during movement (0x%04X)", status_word_read);
             check_and_clear_error(); // diagnose error
             break;
        }
    
        // Read actual position
        int32_t actual_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
        int32_t current_diff = (position > actual_pos) ? (position - actual_pos) : (actual_pos - position);
        
        // Check target reached bit (bit 10) or sufficiently small position error
        if ((status_word_read & 0x0400) || current_diff < 100)
        {
            RCLCPP_INFO(this->get_logger(), "🎯 Target position reached (status word: 0x%04X, position difference: %d)", status_word_read, current_diff);
            break;
        }
        
        // Log progress every 5 retries
        if (retry % 5 == 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "😑  Waiting for target to reach... (retry %d/%d, position difference: %d)", retry, max_retries, current_diff);
        }
        
        retry++;
    }
    
    if (retry >= max_retries)
    {
        int32_t final_pos = read_sdo(OD_ACTUAL_POSITION, 0x00);
        int32_t final_diff = (position > final_pos) ? (position - final_pos) : (final_pos - position);
        RCLCPP_WARN(this->get_logger(), "Timeout waiting for target position to reach (final position difference: %d command units, approximately %.2f°)", 
                   final_diff, position_to_angle(final_diff));
    }
}

void CANopenROS2::set_velocity(float velocity_deg_per_sec)
{
    RCLCPP_INFO(this->get_logger(), "🦿 Setting velocity: %.2f°/s", velocity_deg_per_sec);
    
    // read the current operation mode
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    
    if (mode != MODE_PROFILE_VELOCITY)
    {
        RCLCPP_WARN(this->get_logger(), "👀 Not in velocity mode, cannot set velocity. Current mode: %d", mode);
        return;
    }
    
    // convert to command units
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    
    // set target velocity
    write_sdo(0x60FF, 0x00, velocity_units, 4);  // 0x60FF is the target velocity object
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // enable operation
    write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
    
    RCLCPP_INFO(this->get_logger(), "🦿 Velocity set: %.2f°/s (Command units: %d)", velocity_deg_per_sec, velocity_units);
}

void CANopenROS2::set_velocity_pdo(float velocity_deg_per_sec)
{
    RCLCPP_INFO(this->get_logger(), "✍️ Setting velocity using PDO: %.2f°/s", velocity_deg_per_sec);
    
    // read the current operation mode
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "🤞 Current operation mode: %d", mode);
    
    // if the current mode is not velocity mode, need to switch to velocity mode
    if (mode != MODE_PROFILE_VELOCITY)
    {
        RCLCPP_INFO(this->get_logger(), "🫸🫸Current mode is not velocity mode, switching to velocity mode...");
        
        // try to switch to velocity mode (MODE_PROFILE_VELOCITY = 3)
        set_operation_mode(MODE_PROFILE_VELOCITY);
        
        // check the mode again
        mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "🤝 Operation mode after switch: %d", mode);
        
        if (mode != MODE_PROFILE_VELOCITY)
        {
            RCLCPP_ERROR(this->get_logger(), "🫸🫸Unable to switch to velocity mode, current mode: %d. Velocity setting may fail.", mode);
            // try to set velocity again, but may fail
        }
    }
    
    // set profile velocity parameters (for velocity mode)
    set_profile_velocity(velocity_deg_per_sec);
    
    // convert to command units
    int32_t velocity_units = velocity_to_units(velocity_deg_per_sec);
    
    // ensure the motor is in operation enabled state
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    uint16_t control_word = CONTROL_ENABLE_OPERATION;
    
    if ((status_word & 0x006F) != 0x0027)  // if not in operation enabled state
    {
        RCLCPP_INFO(this->get_logger(), "👀 Motor not in operation state, enabling...");
        // state machine transition
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        write_sdo(OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Write the target velocity via SDO as a compatibility fallback.
    // Some drives accept 0x60FF reliably through SDO even when PDO timing is finicky.
    write_sdo(OD_TARGET_VELOCITY, 0x00, velocity_units, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // send control word and target velocity using RxPDO2 (real-time control)
    struct can_frame frame;
    frame.can_id = COB_RPDO2 + node_id_;
    frame.can_dlc = 6;  // control word (2 bytes) + target velocity (4 bytes)
    
    // control word (0x6040) - 2 bytes
    frame.data[0] = control_word & 0xFF;  // control word low byte
    frame.data[1] = (control_word >> 8) & 0xFF;  // control word high byte
    
    // target velocity (0x60FF) - 4 bytes (little-endian)
    frame.data[2] = velocity_units & 0xFF;
    frame.data[3] = (velocity_units >> 8) & 0xFF;
    frame.data[4] = (velocity_units >> 16) & 0xFF;
    frame.data[5] = (velocity_units >> 24) & 0xFF;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "🙅‍♀️ 🙅‍♀️ Failed to send velocity PDO [Node ID=%d]", node_id_);
    }
    else
    {
        send_sync_frame();
        RCLCPP_DEBUG(this->get_logger(), "🦸‍♀️ ✍️ Velocity PDO sent [Node ID=%d]: %.2f°/s (Command units: %d)", 
                    node_id_, velocity_deg_per_sec, velocity_units);
    }
    
    RCLCPP_INFO(this->get_logger(), "👣 🤟Velocity command sent: %.2f°/s (Command units: %d)", velocity_deg_per_sec, velocity_units);
}
