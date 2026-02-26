#include "crane_master/canopen_ros2_node.hpp"
#include <errno.h>
#include <cmath>

CANopenROS2::CANopenROS2() : Node("canopen_ros2")
{
    // Declare parameters
    this->declare_parameter<std::string>("can_interface");
    this->declare_parameter<std::string>("node_id");
    this->declare_parameter<float>("gear_ratio");
    this->declare_parameter<int>("target_units_per_rev");
    this->declare_parameter<float>("profile_velocity");
    this->declare_parameter<float>("profile_acceleration");
    this->declare_parameter<float>("profile_deceleration");
    this->declare_parameter<int>("position_range_limit_max");
    this->declare_parameter<int>("position_range_limit_min");
    
    // Read parameters
    can_interface_ = this->get_parameter("can_interface").as_string();
    std::string node_id_str = this->get_parameter("node_id").as_string();
    gear_ratio_ = this->get_parameter("gear_ratio").as_double();
    target_units_per_rev_ = this->get_parameter("target_units_per_rev").as_int();
    profile_velocity_ = static_cast<float>(this->get_parameter("profile_velocity").as_double());
    profile_acceleration_ = static_cast<float>(this->get_parameter("profile_acceleration").as_double());
    profile_deceleration_ = static_cast<float>(this->get_parameter("profile_deceleration").as_double());
    position_range_limit_max_ = static_cast<int32_t>(this->get_parameter("position_range_limit_max").as_int());
    position_range_limit_min_ = static_cast<int32_t>(this->get_parameter("position_range_limit_min").as_int());
    
    // Calculate and cache conversion ratios
    // Formula: (angle / 360°) × (target_units_per_rev_ / gear_ratio_) = command units
    // Example: 90° → (90/360) × (10000/10) = 0.25 × 1000 = 250 units
    units_per_degree_ = (static_cast<float>(target_units_per_rev_) / gear_ratio_) / 360.0f;
    degrees_per_unit_ = (gear_ratio_ / static_cast<float>(target_units_per_rev_)) * 360.0f;
    
    // Log the calculated values for debugging
    std::pair<uint32_t, uint32_t> gear_params = calculate_gear_ratio_params(gear_ratio_, target_units_per_rev_);
    RCLCPP_INFO(this->get_logger(), "⚙️ Physical gear ratio: %.2f, Target units/rev: %d, Electronic gear ratio params: (%u, %u), Units/degree: %.6f", 
               gear_ratio_, target_units_per_rev_, gear_params.first, gear_params.second, units_per_degree_,
               gear_ratio_, target_units_per_rev_, gear_params.first, gear_params.second, units_per_degree_);
    
    // Debug: print read parameter values
    RCLCPP_INFO(this->get_logger(), "🔍 [DEBUG] Node name: %s, Read node_id parameter value: '%s', Gear ratio: %.2f", 
                 this->get_name(), node_id_str.c_str(), gear_ratio_,
                 this->get_name(), node_id_str.c_str(), gear_ratio_);
    
    // Convert node_id string to integer
    try {
        node_id_ = std::stoi(node_id_str);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to parse node_id parameter '%s', trying to extract from node name", node_id_str.c_str(), node_id_str.c_str());
        // If parameter parsing fails, try to extract from node name (e.g., canopen_ros2_node1 -> 1)
        std::string node_name = this->get_name();
        size_t node_pos = node_name.find("node");
        if (node_pos != std::string::npos) {
            size_t num_start = node_pos + 4; // "node" is 4 characters
            if (num_start < node_name.length()) {
                try {
                    node_id_ = std::stoi(node_name.substr(num_start));
                    RCLCPP_INFO(this->get_logger(), "✅ Extracted node ID %d from node name '%s'", node_id_, node_name.c_str(), node_id_, node_name.c_str());
                } catch (...) {
                    node_id_ = 1;
                    RCLCPP_WARN(this->get_logger(), "⚠️ Failed to extract node ID from node name, using default value 1");
                }
            } else {
                node_id_ = 1;
            }
        } else {
            node_id_ = 1;
        }
    }
    
    // If the value read from parameters seems wrong (e.g., all nodes read 2), try to extract from node name
    std::string node_name = this->get_name();
    // Extract node ID from node name (e.g., canopen_ros2_node1 -> 1)
    size_t node_pos = node_name.find("node");
    if (node_pos != std::string::npos) {
        size_t num_start = node_pos + 4; // "node" is 4 characters
        if (num_start < node_name.length()) {
            try {
                int extracted_id = std::stoi(node_name.substr(num_start));
                if (extracted_id >= 1 && extracted_id <= 3 && extracted_id != node_id_) {
                    RCLCPP_WARN(this->get_logger(), "⚠️ Node name '%s' indicates node ID should be %d, but parameter value is %d, using node name value", 
                               node_name.c_str(), extracted_id, node_id_,
                               node_name.c_str(), extracted_id, node_id_);
                    node_id_ = extracted_id;
                }
            } catch (...) {
                // Ignore extraction failure
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "🏗️ Initializing tower crane control, node name=%s, CAN interface=%s, node ID=%d (original parameter value: '%s'), gear ratio=%.2f", 
                this->get_name(), can_interface_.c_str(), node_id_, node_id_str.c_str(), gear_ratio_,
                this->get_name(), can_interface_.c_str(), node_id_, node_id_str.c_str(), gear_ratio_);
    
    // Initialize CAN socket
    init_can_socket();
    
    if (can_socket_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ CAN socket initialization failed, cannot continue");
        return;
    }
    
    // Create publishers, configure QoS to avoid RTPS payload size errors
    // Must be created before initializing node, as receive_can_frames may be called during initialization
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    
    // String messages need larger payload size
    status_pub_ = this->create_publisher<std_msgs::msg::String>("crane_status", qos_profile);
    // Create position publisher: publishes current tower crane position (Float32 type, unit: degrees), topic name "crane_position", queue size 10
    // Create position publisher: publishes current crane position (Float32 type, unit: degrees), topic name "crane_position", queue size 10
    position_pub_ = this->create_publisher<std_msgs::msg::Float32>("crane_position", 10);
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("crane_velocity", 10);
    
    // Create subscribers (using relative paths, supports namespaces)
    position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "target_position", 10, std::bind(&CANopenROS2::position_callback, this, std::placeholders::_1));
    velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "target_velocity", 10, std::bind(&CANopenROS2::velocity_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "📥 Subscribers created: target_position, target_velocity");
    
    // Create services
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "start_crane", std::bind(&CANopenROS2::handle_start, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_crane", std::bind(&CANopenROS2::handle_stop, this, std::placeholders::_1, std::placeholders::_2));
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_crane", std::bind(&CANopenROS2::handle_reset, this, std::placeholders::_1, std::placeholders::_2));
    set_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_crane_mode", std::bind(&CANopenROS2::handle_set_mode, this, std::placeholders::_1, std::placeholders::_2));
    
    // Initialize node (before creating timer, to avoid timer callbacks running during initialization)
    initialize_node();
    
    // Configure PDO mapping
    configure_pdo();
    
    // Wait for a period
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Start node
    start_node();
    
    // Set immediate effect
    set_immediate_effect(true);
    
    // Clear faults
    clear_fault();
    
    // Enable motor
    enable_motor();
    
    // Read and record gear ratio (0x6091:01 and 0x6091:02) - read only for reference, do not modify
    int32_t motor_revolutions = read_sdo(OD_GEAR_RATIO, 0x01);
    int32_t shaft_revolutions = read_sdo(OD_GEAR_RATIO, 0x02);
    
    if (motor_revolutions > 0 && shaft_revolutions > 0)
    {
        float calculated_gear_ratio = static_cast<float>(motor_revolutions) / static_cast<float>(shaft_revolutions);
        RCLCPP_INFO(this->get_logger(), "⚙️ Current gear ratio (0x6091): Motor revolutions=%d, Shaft revolutions=%d, Calculated value=%.2f (Configured value=%.2f)", 
                   motor_revolutions, shaft_revolutions, calculated_gear_ratio, gear_ratio_,
                   motor_revolutions, shaft_revolutions, calculated_gear_ratio, gear_ratio_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "⚠️ Unable to read or invalid gear ratio (0x6091), using configured value: %.2f", gear_ratio_, gear_ratio_);
    }
    
    // Set position range limit (0x607B sub1=max, sub2=min)
    set_position_range_limit(position_range_limit_max_, position_range_limit_min_);

    // Set target position (e.g., move to 90 degrees)
    go_to_position(0.0);
    
    // Now all initialization is complete, create timer for receiving CAN frames
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CANopenROS2::receive_can_frames, this));
    
    // Create status timer
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CANopenROS2::publish_status, this));
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

