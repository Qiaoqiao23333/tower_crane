#include "crane_master/canopen_ros2_node.hpp"

void CANopenROS2::publish_status()
{
    // 发布状态信息
    auto status_msg = std_msgs::msg::String();
    
    // 根据状态字解析状态
    std::string status_str = "未知";
    if (status_word_ & 0x0008)  // 故障
    {
        status_str = "故障";
    }
    else if ((status_word_ & 0x006F) == 0x0027)  // 操作已启用
    {
        status_str = "操作已启用";
    }
    else if ((status_word_ & 0x006F) == 0x0023)  // 已开启
    {
        status_str = "已开启";
    }
    else if ((status_word_ & 0x006F) == 0x0021)  // 准备开启
    {
        status_str = "准备开启";
    }
    else if ((status_word_ & 0x004F) == 0x0040)  // 禁止开启
    {
        status_str = "禁止开启";
    }
    
    status_msg.data = status_str;
    status_pub_->publish(status_msg);
    
    // 发布位置
    auto pos_msg = std_msgs::msg::Float32();
    pos_msg.data = position_to_angle(position_);
    position_pub_->publish(pos_msg);
}

void CANopenROS2::position_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float angle = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到目标位置: %.2f°\nReceived target position: %.2f°", angle, angle);
    
    // 添加更多调试信息
    RCLCPP_INFO(this->get_logger(), "当前CAN套接字: %d\nCurrent CAN socket: %d", can_socket_, can_socket_);
    RCLCPP_INFO(this->get_logger(), "当前节点ID: %d\nCurrent node ID: %d", node_id_, node_id_);
    
    // 读取当前状态字
    int32_t status_word = read_sdo(OD_STATUS_WORD, 0x00);
    RCLCPP_INFO(this->get_logger(), "当前状态字: 0x%04X\nCurrent status word: 0x%04X", status_word, status_word);
    
    // 读取当前操作模式
    int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
    RCLCPP_INFO(this->get_logger(), "当前操作模式: %d", mode);
    
    go_to_position(angle);
}

void CANopenROS2::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float velocity = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到目标速度: %.2f°/s\nReceived target velocity: %.2f°/s", velocity, velocity);
    
    // 尝试使用PDO设置速度
    set_velocity_pdo(velocity);
}

void CANopenROS2::handle_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到启动请求\nReceived start request");
    
    try
    {
        initialize_motor();
        response->success = true;
        response->message = "crane电机已启动";
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
    RCLCPP_INFO(this->get_logger(), "收到停止请求\nReceived stop request");
    
    try
    {
        stop_motor();
        response->success = true;
        response->message = "crane电机已停止";
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
    RCLCPP_INFO(this->get_logger(), "收到重置请求\nReceived reset request");
    
    try
    {
        // 发送NMT重置命令
        send_nmt_command(NMT_RESET_NODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 重新初始化电机
        initialize_motor();
        
        response->success = true;
        response->message = "crane电机已重置";
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
    RCLCPP_INFO(this->get_logger(), "收到设置模式请求: %s\nReceived set mode request: %s", request->data ? "位置模式" : "速度模式", request->data ? "Position mode" : "Velocity mode");
    
    try
    {
        // 读取当前操作模式
        int32_t mode = read_sdo(OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "当前操作模式: %d\nCurrent operation mode: %d", mode, mode);
        
        // 无论当前模式如何，都设置相应的参数
        if (request->data)
        {
            // 设置位置模式参数
            set_profile_parameters(30, 30, 30);  // 速度、加速度、减速度：30°/s, 30°/s², 30°/s²
            
            // 设置目标位置为当前位置，防止电机立即运动
            int32_t current_position = read_sdo(OD_ACTUAL_POSITION, 0x00);
            write_sdo(OD_TARGET_POSITION, 0x00, current_position, 4);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            response->message = "已设置位置模式参数";
        }
        else
        {
            // 设置速度模式参数
            set_profile_velocity(30);  // 默认速度：30°/s
            
            // 设置目标速度为0，防止电机立即运动
            write_sdo(0x60FF, 0x00, 0, 4);  // 0x60FF是目标速度对象
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            response->message = "已设置速度模式参数";
        }
        
        response->success = true;
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = "设置模式参数失败: " + std::string(e.what());
    }
}

