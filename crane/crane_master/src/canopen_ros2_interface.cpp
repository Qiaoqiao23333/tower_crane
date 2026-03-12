#include "crane_master/canopen_ros2_node.hpp"

void CANopenROS2::publish_status()
{
    // Publish status information
    auto status_msg = std_msgs::msg::String();
    
    // Decode status word into human-readable string
    std::string status_str = "Unknown";
    if (status_word_ & 0x0008)  // fault
    {
        status_str = "Fault";
    }
    else if ((status_word_ & 0x006F) == 0x0027)  // operation enabled
    {
        status_str = "Operation enabled";
    }
    else if ((status_word_ & 0x006F) == 0x0023)  // switched on
    {
        status_str = "Switched on";
    }
    else if ((status_word_ & 0x006F) == 0x0021)  // ready to switch on
    {
        status_str = "Ready to switch on";
    }
    else if ((status_word_ & 0x004F) == 0x0040)  // operation inhibit
    {
        status_str = "Operation inhibit";
    }
    
    status_msg.data = status_str;
    status_pub_->publish(status_msg);
    
    // Publish position
    auto pos_msg = std_msgs::msg::Float32();
    pos_msg.data = position_to_angle(position_);
    position_pub_->publish(pos_msg);
    
    // Publish velocity (computed via position difference to avoid SDO blocking)
    rclcpp::Time now = this->now();
    double dt = (now - prev_position_time_).seconds();
    auto vel_msg = std_msgs::msg::Float32();
    if (dt > 0.0)
    {
        float delta_deg = static_cast<float>(position_ - prev_position_) * degrees_per_unit_;
        vel_msg.data = delta_deg / static_cast<float>(dt);
    }
    else
    {
        vel_msg.data = 0.0f;
    }
    prev_position_ = position_;
    prev_position_time_ = now;
    velocity_pub_->publish(vel_msg);
}

void CANopenROS2::position_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float angle = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received target position: %.2f°", angle);
    
    // Additional debug information
    RCLCPP_INFO(this->get_logger(), "Current CAN socket: %d", can_socket_);
    RCLCPP_INFO(this->get_logger(), "Current node ID: %d", node_id_);
    
    // Read current status word
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "Current status word: 0x%04X", status_word);
    
    // Read current operation mode
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "Current operation mode: %d", mode);
    
    go_to_position(angle);
}

void CANopenROS2::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float velocity = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received target velocity: %.2f°/s", velocity);
    
    // Try to set velocity using PDO
    set_velocity_pdo(velocity);
}

void CANopenROS2::handle_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received start request");
    
    try
    {
        initialize_motor();
        response->success = true;
        response->message = "Crane motor started";
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = "启动失败: " + std::string(e.what());
    }
}

void CANopenROS2::handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received stop request");
    
    try
    {
        stop_motor();
        response->success = true;
        response->message = "Crane motor stopped";
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = "停止失败: " + std::string(e.what());
    }
}

void CANopenROS2::handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received reset request");
    
    try
    {
        // Send NMT reset command
        send_nmt_command(NMT_RESET_NODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Reinitialize motor
        initialize_motor();
        
        response->success = true;
        response->message = "Crane motor reset";
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = "重置失败: " + std::string(e.what());
    }
}

void CANopenROS2::handle_set_mode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received set mode request: %s", request->data ? "Position mode" : "Velocity mode");
    
    try
    {
        // Read current operation mode
        int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "Current operation mode: %d", mode);
        
        // Set appropriate parameters regardless of current mode
        if (request->data)
        {
            // Configure position mode parameters
            set_profile_parameters(profile_velocity_, profile_acceleration_, profile_deceleration_);
            
            // Set target position to current position to prevent immediate motion
            int32_t current_position = read_sdo(OD_ACTUAL_POSITION, 0x00);
            write_sdo(OD_TARGET_POSITION, 0x00, current_position, 4);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            response->message = "Position mode parameters configured";
        }
        else
        {
            // Configure velocity mode parameters
            set_profile_velocity(profile_velocity_);
            
            // Set target velocity to 0 to prevent immediate motion
            write_sdo(0x60FF, 0x00, 0, 4);  // 0x60FF is target velocity object
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            response->message = "Velocity mode parameters configured";
        }
        
        response->success = true;
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = "设置模式参数失败: " + std::string(e.what());
    }
}

