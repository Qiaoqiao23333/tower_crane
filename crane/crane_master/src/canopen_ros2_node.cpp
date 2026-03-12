#include "crane_master/canopen_ros2_node.hpp"
#include <errno.h>
#include <cmath>

CANopenROS2::CANopenROS2() : Node("canopen_ros2")
{
    // Declare parameters
    this->declare_parameter<std::string>("can_interface_name", "can0");
    this->declare_parameter<std::string>("node_id", "1");
    this->declare_parameter<float>("gear_ratio", 1.0);
    this->declare_parameter<int>("target_units_per_rev", 10000);
    this->declare_parameter<bool>("auto_start", true);
    this->declare_parameter<float>("profile_velocity", 30.0);
    this->declare_parameter<float>("profile_acceleration", 30.0);
    this->declare_parameter<float>("profile_deceleration", 30.0);
    this->declare_parameter<int>("cycle_period_us", 1000);
    
    // Read parameters
    can_interface_ = this->get_parameter("can_interface_name").as_string();
    std::string node_id_str = this->get_parameter("node_id").as_string();
    gear_ratio_ = this->get_parameter("gear_ratio").as_double();
    target_units_per_rev_ = this->get_parameter("target_units_per_rev").as_int();
    auto_start_ = this->get_parameter("auto_start").as_bool();
    profile_velocity_ = this->get_parameter("profile_velocity").as_double();
    profile_acceleration_ = this->get_parameter("profile_acceleration").as_double();
    profile_deceleration_ = this->get_parameter("profile_deceleration").as_double();
    cycle_period_us_ = this->get_parameter("cycle_period_us").as_int();
    
    // Compute and cache conversion ratios
    // Formula: (angle / 360°) × (target_units_per_rev_ / gear_ratio_) = command units
    // Example: 90 degrees → (90/360) × (10000/10) = 0.25 × 1000 = 250 units
    units_per_degree_ = (static_cast<float>(target_units_per_rev_) / gear_ratio_) / 360.0f;
    degrees_per_unit_ = (gear_ratio_ / static_cast<float>(target_units_per_rev_)) * 360.0f;
    
    // Log the calculated values for debugging
    std::pair<uint32_t, uint32_t> gear_params = calculate_gear_ratio_params(gear_ratio_, target_units_per_rev_);
    RCLCPP_INFO(this->get_logger(), "Physical gear ratio: %.2f, target units/rev: %d, electronic gear ratio params: (%u, %u), units/degree: %.6f", 
               gear_ratio_, target_units_per_rev_, gear_params.first, gear_params.second, units_per_degree_);
    
    // Debug: print raw parameter values
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Node name: %s, read node_id parameter value: '%s', gear ratio: %.2f", 
                 this->get_name(), node_id_str.c_str(), gear_ratio_);
    
    // Convert node_id string to integer
    try {
        node_id_ = std::stoi(node_id_str);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse node_id parameter '%s', attempting to extract from node name", node_id_str.c_str());
        // If parsing fails, try extracting from node name (e.g., canopen_ros2_node1 -> 1)
        std::string node_name = this->get_name();
        size_t node_pos = node_name.find("node");
        if (node_pos != std::string::npos) {
            size_t num_start = node_pos + 4; // "node" is 4 characters
            if (num_start < node_name.length()) {
                try {
                    node_id_ = std::stoi(node_name.substr(num_start));
                    RCLCPP_INFO(this->get_logger(), "Extracted node ID: %d from node name '%s'", node_id_, node_name.c_str());
                } catch (...) {
                    node_id_ = 1;
                    RCLCPP_WARN(this->get_logger(), "Failed to extract node ID from node name, using default value 1");
                }
            } else {
                node_id_ = 1;
            }
        } else {
            node_id_ = 1;
        }
    }
    
    // If parameter value looks suspicious (e.g. all nodes read as the same), try extracting from node name
    std::string node_name = this->get_name();
    // Extract node ID from node name (e.g., canopen_ros2_node1 -> 1)
    size_t node_pos = node_name.find("node");
    if (node_pos != std::string::npos) {
        size_t num_start = node_pos + 4; // "node" is 4 characters
        if (num_start < node_name.length()) {
            try {
                int extracted_id = std::stoi(node_name.substr(num_start));
                if (extracted_id >= 1 && extracted_id <= 3 && extracted_id != node_id_) {
                    RCLCPP_WARN(this->get_logger(), "Node name '%s' indicates node ID should be %d, but parameter value is %d, using node name value", 
                               node_name.c_str(), extracted_id, node_id_);
                    node_id_ = extracted_id;
                }
            } catch (...) {
                // 忽略提取失败
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Initializing Simple crane Control, node name=%s, CAN interface=%s, node ID=%d (original parameter value: '%s'), gear ratio=%.2f", 
                this->get_name(), can_interface_.c_str(), node_id_, node_id_str.c_str(), gear_ratio_);
    
    // Initialize CAN socket
    init_can_socket();
    
    if (can_socket_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "CAN socket initialization failed, cannot continue");
        return;
    }
    
    // Create publishers and configure QoS to avoid RTPS payload size errors
    // Must be created before node initialization, as receive_can_frames may be called during init
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    
    // String messages need larger payload size
    status_pub_ = this->create_publisher<std_msgs::msg::String>("crane_status", qos_profile);
    position_pub_ = this->create_publisher<std_msgs::msg::Float32>("crane_position", 10);
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("crane_velocity", 10);
    
    // Create subscriptions (relative topics so namespaces work)
    position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "target_position", 10, std::bind(&CANopenROS2::position_callback, this, std::placeholders::_1));
    velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "target_velocity", 10, std::bind(&CANopenROS2::velocity_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribers created: target_position, target_velocity");
    
    // Create services
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "start_crane", std::bind(&CANopenROS2::handle_start, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_crane", std::bind(&CANopenROS2::handle_stop, this, std::placeholders::_1, std::placeholders::_2));
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_crane", std::bind(&CANopenROS2::handle_reset, this, std::placeholders::_1, std::placeholders::_2));
    set_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_crane_mode", std::bind(&CANopenROS2::handle_set_mode, this, std::placeholders::_1, std::placeholders::_2));
    
    // Initialize node (before creating timers to avoid callbacks during init)
    initialize_node();
    
    // Configure PDO mapping
    configure_pdo();
    
    // Wait a short period
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Start node
    start_node();
    
    // Set immediate effect
    set_immediate_effect(true);
    
    // Clear faults
    clear_fault();
    
    // Enable motor
    enable_motor();
    
    // Read and log gear ratio (0x6091:01 and 0x6091:02) - read-only for reference
    int32_t motor_revolutions = read_sdo(OD_GEAR_RATIO, 0x01);
    int32_t shaft_revolutions = read_sdo(OD_GEAR_RATIO, 0x02);
    
    if (motor_revolutions > 0 && shaft_revolutions > 0)
    {
        float calculated_gear_ratio = static_cast<float>(motor_revolutions) / static_cast<float>(shaft_revolutions);
        RCLCPP_INFO(this->get_logger(), "Current gear ratio (0x6091): motor revolutions=%d, shaft revolutions=%d, calculated value=%.2f (configured value=%.2f)", 
                   motor_revolutions, shaft_revolutions, calculated_gear_ratio, gear_ratio_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Unable to read or invalid gear ratio (0x6091), using configured value: %.2f", gear_ratio_);
    }
    
    // Initialize baseline time and position for velocity computation
    prev_position_time_ = this->now();
    prev_position_ = position_;
    
    // With initialization complete, create timer to receive CAN frames
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CANopenROS2::receive_can_frames, this));
    
    // Create status timer (separate callback group to avoid being blocked)
    status_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CANopenROS2::publish_status, this),
        status_cb_group_);
}

CANopenROS2::~CANopenROS2()
{
    // Stop motor
    stop_motor();
    
    // Close CAN socket
    if (can_socket_ >= 0)
    {
        close(can_socket_);
    }
}

