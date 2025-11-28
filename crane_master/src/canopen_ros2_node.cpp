#include "crane_master/canopen_ros2_node.hpp"
#include <errno.h>

CANopenROS2::CANopenROS2() : Node("canopen_ros2")
{
    // 声明参数
    this->declare_parameter<std::string>("can_interface", "can0");
    this->declare_parameter<std::string>("node_id", "1");
    
    // 读取参数
    can_interface_ = this->get_parameter("can_interface").as_string();
    std::string node_id_str = this->get_parameter("node_id").as_string();
    
    // 调试：打印读取到的原始参数值
    RCLCPP_INFO(this->get_logger(), "[DEBUG] 节点名称: %s, 读取到的node_id参数值: '%s'", 
                 this->get_name(), node_id_str.c_str());
    
    // 将node_id字符串转换为整数
    try {
        node_id_ = std::stoi(node_id_str);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "无法解析node_id参数 '%s'，尝试从节点名称提取", node_id_str.c_str());
        // 如果参数解析失败，尝试从节点名称提取（例如：canopen_ros2_node1 -> 1）
        std::string node_name = this->get_name();
        size_t node_pos = node_name.find("node");
        if (node_pos != std::string::npos) {
            size_t num_start = node_pos + 4; // "node" is 4 characters
            if (num_start < node_name.length()) {
                try {
                    node_id_ = std::stoi(node_name.substr(num_start));
                    RCLCPP_INFO(this->get_logger(), "从节点名称 '%s' 提取到节点ID: %d", node_name.c_str(), node_id_);
                } catch (...) {
                    node_id_ = 1;
                    RCLCPP_WARN(this->get_logger(), "无法从节点名称提取节点ID，使用默认值1");
                }
            } else {
                node_id_ = 1;
            }
        } else {
            node_id_ = 1;
        }
    }
    
    // 如果从参数读取的值看起来不对（比如所有节点都读到2），尝试从节点名称提取
    std::string node_name = this->get_name();
    // 从节点名称提取节点ID（例如：canopen_ros2_node1 -> 1）
    size_t node_pos = node_name.find("node");
    if (node_pos != std::string::npos) {
        size_t num_start = node_pos + 4; // "node" is 4 characters
        if (num_start < node_name.length()) {
            try {
                int extracted_id = std::stoi(node_name.substr(num_start));
                if (extracted_id >= 1 && extracted_id <= 3 && extracted_id != node_id_) {
                    RCLCPP_WARN(this->get_logger(), "节点名称 '%s' 指示节点ID应为%d，但参数值为%d，使用节点名称的值", 
                               node_name.c_str(), extracted_id, node_id_);
                    node_id_ = extracted_id;
                }
            } catch (...) {
                // 忽略提取失败
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "初始化Simple crane Control，节点名称=%s，CAN接口=%s，节点ID=%d (原始参数值: '%s')", 
                this->get_name(), can_interface_.c_str(), node_id_, node_id_str.c_str());
    
    // 初始化CAN套接字
    init_can_socket();
    
    if (can_socket_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "CAN套接字初始化失败，无法继续");
        return;
    }
    
    // 创建发布器，配置QoS以避免RTPS payload size错误
    // 必须在初始化节点之前创建，因为receive_can_frames可能会在初始化过程中被调用
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    
    // String消息需要更大的payload size
    status_pub_ = this->create_publisher<std_msgs::msg::String>("crane_status", qos_profile);
    position_pub_ = this->create_publisher<std_msgs::msg::Float32>("crane_position", 10);
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("crane_velocity", 10);
    
    // 创建订阅器（使用绝对路径确保在全局命名空间）
    position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/target_position", 10, std::bind(&CANopenROS2::position_callback, this, std::placeholders::_1));
    velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/target_velocity", 10, std::bind(&CANopenROS2::velocity_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "订阅器已创建: /target_position, /target_velocity");
    
    // 创建服务
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "start_crane", std::bind(&CANopenROS2::handle_start, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_crane", std::bind(&CANopenROS2::handle_stop, this, std::placeholders::_1, std::placeholders::_2));
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_crane", std::bind(&CANopenROS2::handle_reset, this, std::placeholders::_1, std::placeholders::_2));
    set_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_crane_mode", std::bind(&CANopenROS2::handle_set_mode, this, std::placeholders::_1, std::placeholders::_2));
    
    // 初始化节点（在创建定时器之前，避免定时器回调在初始化期间运行）
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
    
    // 设置目标位置（例如，移动到90度）
    go_to_position(0.0);
    
    // 现在所有初始化完成，创建定时器用于接收CAN帧
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CANopenROS2::receive_can_frames, this));
    
    // 创建状态定时器
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CANopenROS2::publish_status, this));
}

CANopenROS2::~CANopenROS2()
{
    // 停止电机
    stop_motor();
    
    // 关闭CAN套接字
    if (can_socket_ >= 0)
    {
        close(can_socket_);
    }
}

