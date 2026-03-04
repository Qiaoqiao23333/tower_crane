#include "crane_master/canopen_ros2_node.hpp"

void CANopenROS2::publish_status()
{
    // Publish status information
    auto status_msg = std_msgs::msg::String();
    
    // Parse status based on status word
    std::string status_str = "Unknown";
    if (status_word_ & 0x0008)  // Fault
    {
        status_str = "Fault";
    }
    else if ((status_word_ & 0x006F) == 0x0027)  // Operation enabled
    {
        status_str = "Operation enabled";
    }
    else if ((status_word_ & 0x006F) == 0x0023)  // Switched on
    {
        status_str = "Switched on";
    }
    else if ((status_word_ & 0x006F) == 0x0021)  // Ready to switch on
    {
        status_str = "Ready to switch on";
    }
    else if ((status_word_ & 0x004F) == 0x0040)  // Operation inhibit
    {
        status_str = "Operation inhibit";
    }
    
    status_msg.data = status_str;
    status_pub_->publish(status_msg);
    
    // Publish position
    auto pos_msg = std_msgs::msg::Float32();
    pos_msg.data = position_to_angle(position_);
    position_pub_->publish(pos_msg);
    
    // Read and publish velocity (read actual velocity via SDO 0x606C)
    try
    {
        int32_t velocity_units = read_sdo(OD_ACTUAL_VELOCITY, 0x00);
        velocity_ = velocity_units;
        float velocity_deg_per_sec = units_to_velocity(velocity_units);
        
        // Publish velocity (check if publisher is initialized)
        if (velocity_pub_)
        {
            auto vel_msg = std_msgs::msg::Float32();
            vel_msg.data = velocity_deg_per_sec;
            velocity_pub_->publish(vel_msg);
        }
    }
    catch (...)
    {
        // If read fails, publish 0 velocity or skip
        if (velocity_pub_)
        {
            auto vel_msg = std_msgs::msg::Float32();
            vel_msg.data = 0.0;
            velocity_pub_->publish(vel_msg);
        }
    }
}

void CANopenROS2::position_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float angle = msg->data;
    RCLCPP_INFO(this->get_logger(), "📍 Received target position: %.4f r (output shaft revolutions)", angle);
    
    // Add more debug information
    RCLCPP_INFO(this->get_logger(), "🔌 Current CAN socket: %d", can_socket_);
    RCLCPP_INFO(this->get_logger(), "🆔 Current node ID: %d", node_id_);
    
    // Read current status word
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "👉🏼 Current status word: 0x%04X", status_word);
    
    // Read current operation mode
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "👉🏼Current operation mode: %d", mode);
    
    // If drive is in fault state, try to clear and re-enable before moving
    if (status_word & 0x0008)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Drive in fault state (0x%04X), attempting recovery...", status_word);
        clear_fault();
        enable_motor();
        
        // Re-check status after recovery attempt
        status_word = read_sdo(OD_STATUS_WORD, 0x00);
        if (status_word & 0x0008)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to clear fault (0x%04X), cannot execute position command", status_word);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ Fault cleared, status word: 0x%04X", status_word);
    }
    
    go_to_position(angle);
}

void CANopenROS2::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float velocity = msg->data;
    RCLCPP_INFO(this->get_logger(), "🏃 Received target velocity: %.2f°/s", velocity);
    
    // Try to set velocity using PDO
    set_velocity_pdo(velocity);
}

void CANopenROS2::handle_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "✋🏼 Received start request");
    
    try
    {
        initialize_motor();
        response->success = true;
        response->message = "Crane motor started";
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = "Start failed: " + std::string(e.what());
    }
}

void CANopenROS2::handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "✋🏼 Received stop request");
    
    try
    {
        stop_motor();
        response->success = true;
        response->message = "Crane motor stopped";
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = "Stop failed: " + std::string(e.what());
    }
}

void CANopenROS2::handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "🔄 Received reset request");
    
    try
    {
        // Send NMT reset command
        send_nmt_command(NMT_RESET_NODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Re-initialize motor
        initialize_motor();
        
        response->success = true;
        response->message = "Crane motor reset";
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = "Reset failed: " + std::string(e.what());
    }
}

void CANopenROS2::handle_set_mode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "📨 Received set mode request: %s", request->data ? "Position mode" : "Velocity mode");
    
    try
    {
        // Read current operation mode
        int32_t current_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), " 👉🏼Current operation mode: %d", current_mode);
        
        if (request->data)
        {
            // Switch to position mode
            RCLCPP_INFO(this->get_logger(), "👀 Switching to position mode...");
            
            // 1. First set position mode parameters (from ROS2 parameters)
            set_profile_parameters(profile_velocity_, profile_acceleration_, profile_deceleration_);
            
            // 2. Switch to position mode
            set_operation_mode(MODE_PROFILE_POSITION);
            
            // 3. Verify if mode switch was successful
            int32_t new_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
            if (new_mode == MODE_PROFILE_POSITION)
            {
                // Set target position to current position to prevent immediate motor movement
                int32_t current_position = read_sdo(OD_ACTUAL_POSITION, 0x00);
                write_sdo(OD_TARGET_POSITION, 0x00, current_position, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                response->message = "💁🏼‍♀️Successfully switched to position mode";
                response->success = true;
            }
            else
            {
                response->message = "🤦🏼‍♀️Failed to switch to position mode, current mode: " + std::to_string(new_mode);
                response->success = false;
            }
        }
        else
        {
            // Switch to velocity mode
            RCLCPP_INFO(this->get_logger(), "👀 Switching to velocity mode...");
            
            // 1. First set velocity mode parameters (from ROS2 parameter: profile_velocity)
            set_profile_velocity(profile_velocity_);
            
            // 2. Switch to velocity mode
            set_operation_mode(MODE_PROFILE_VELOCITY);
            
            // 3. Verify if mode switch was successful
            int32_t new_mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
            if (new_mode == MODE_PROFILE_VELOCITY)
            {
                // Set target velocity to 0 to prevent immediate motor movement
                write_sdo(0x60FF, 0x00, 0, 4);  // 0x60FF is target velocity object
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                response->message = "💁🏼Successfully switched to velocity mode";
                response->success = true;
            }
            else
            {
                response->message = "🤷🏼Failed to switch to velocity mode, current mode: " + std::to_string(new_mode);
                response->success = false;
            }
        }
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = "Set mode failed: " + std::string(e.what());
    }
}

