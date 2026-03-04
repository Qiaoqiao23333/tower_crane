#include "crane_master/canopen_ros2_node.hpp"
#include <errno.h>
#include <cmath>

CANopenROS2::CANopenROS2() : Node("canopen_ros2")
{
    // Declare parameters
    this->declare_parameter<std::string>("can_interface");
    this->declare_parameter<std::string>("node_id");
    this->declare_parameter<int>("gear_ratio_numerator");
    this->declare_parameter<int>("gear_ratio_denominator");
    this->declare_parameter<float>("max_profile_velocity");
    this->declare_parameter<float>("profile_velocity");
    this->declare_parameter<float>("profile_acceleration");
    this->declare_parameter<float>("profile_deceleration");
    this->declare_parameter<float>("quick_stop_deceleration");
    this->declare_parameter<int>("position_range_limit_max");
    this->declare_parameter<int>("position_range_limit_min");
    
    // Read parameters
    can_interface_ = this->get_parameter("can_interface").as_string();
    std::string node_id_str = this->get_parameter("node_id").as_string();
    gear_ratio_numerator_   = static_cast<uint32_t>(this->get_parameter("gear_ratio_numerator").as_int());
    gear_ratio_denominator_ = static_cast<uint32_t>(this->get_parameter("gear_ratio_denominator").as_int());
    max_profile_velocity_ = static_cast<float>(this->get_parameter("max_profile_velocity").as_double());
    profile_velocity_ = static_cast<float>(this->get_parameter("profile_velocity").as_double());
    profile_acceleration_ = static_cast<float>(this->get_parameter("profile_acceleration").as_double());
    profile_deceleration_ = static_cast<float>(this->get_parameter("profile_deceleration").as_double());
    quick_stop_deceleration_ = static_cast<float>(this->get_parameter("quick_stop_deceleration").as_double());
    position_range_limit_max_ = static_cast<int32_t>(this->get_parameter("position_range_limit_max").as_int());
    position_range_limit_min_ = static_cast<int32_t>(this->get_parameter("position_range_limit_min").as_int());
    
    // NOTE: units_per_degree_ / degrees_per_unit_ are kept for potential future use,
    // but ALL position/velocity/acceleration functions now use a 1:1 mapping:
    //   1 command unit = 1 driving-shaft (output) revolution
    //
    // Drive formula (DSY-C): Motor Position [r] = 0x607A × (6091:01 / 6091:02)
    //   → 0x607A is in output-shaft revolutions; the drive's gear ratio converts to motor revs.
    //   Example: gear_ratio_numerator=10000, gear_ratio_denominator=1 (10 000:1 gearbox)
    //     send data=10.0 → 0x607A=10 → motor moves 10×10000 = 100 000 r → output shaft = 10 r
    //
    // units_per_degree_  = numerator / denominator / 360  (NOT used for position/velocity)
    // degrees_per_unit_  = 360 * denominator / numerator  (NOT used for position/velocity)
    units_per_degree_ = static_cast<double>(gear_ratio_numerator_) / static_cast<double>(gear_ratio_denominator_) / 360.0;
    degrees_per_unit_ = 360.0 * static_cast<double>(gear_ratio_denominator_) / static_cast<double>(gear_ratio_numerator_);
    
    // Log the calculated values for debugging
    RCLCPP_INFO(this->get_logger(), "⚙️ Electronic gear ratio (0x6091): %u/%u, Units/degree: %.6f", 
               gear_ratio_numerator_, gear_ratio_denominator_, units_per_degree_);
    
    // Debug: print read parameter values
    RCLCPP_INFO(this->get_logger(), "🔍 [DEBUG] Node name: %s, Read node_id parameter value: '%s'", 
                 this->get_name(), node_id_str.c_str());
    
    // Convert node_id string to integer
    try {
        node_id_ = std::stoi(node_id_str);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to parse node_id parameter '%s', trying to extract from node name", node_id_str.c_str());
        // If parameter parsing fails, try to extract from node name (e.g., canopen_ros2_node1 -> 1)
        std::string node_name = this->get_name();
        size_t node_pos = node_name.find("node");
        if (node_pos != std::string::npos) {
            size_t num_start = node_pos + 4; // "node" is 4 characters
            if (num_start < node_name.length()) {
                try {
                    node_id_ = std::stoi(node_name.substr(num_start));
                    RCLCPP_INFO(this->get_logger(), "✅ Extracted node ID %d from node name '%s'", node_id_, node_name.c_str());
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
                               node_name.c_str(), extracted_id, node_id_);
                    node_id_ = extracted_id;
                }
            } catch (...) {
                // Ignore extraction failure
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "🏗️ Initializing tower crane control, node name=%s, CAN interface=%s, node ID=%d (original parameter value: '%s')", 
                this->get_name(), can_interface_.c_str(), node_id_, node_id_str.c_str());
    
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
    
    // Read current gear ratio from drive for reference before overwriting
    int32_t motor_revolutions = read_sdo(OD_GEAR_RATIO, 0x01);
    int32_t shaft_revolutions = read_sdo(OD_GEAR_RATIO, 0x02);
    
    if (motor_revolutions > 0 && shaft_revolutions > 0)
    {
        RCLCPP_INFO(this->get_logger(), "⚙️ Current drive gear ratio (0x6091): Motor revolutions=%d, Shaft revolutions=%d", 
                   motor_revolutions, shaft_revolutions);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "⚠️ Unable to read current drive gear ratio (0x6091)");
    }
    
    // Set electronic gear ratio (0x6091 sub1=numerator, sub2=denominator)
    set_electronic_gear_ratio(gear_ratio_numerator_, gear_ratio_denominator_);

    // Set max profile velocity limit (0x607F)
    set_max_profile_velocity(max_profile_velocity_);

    // Set quick stop deceleration (0x6085)
    set_quick_stop_deceleration(quick_stop_deceleration_);

    // Set position range limit (0x607B sub1=max, sub2=min)
    set_position_range_limit(position_range_limit_max_, position_range_limit_min_);

    // Ensure motor stays still: set target velocity to 0 and target position to current position
    RCLCPP_INFO(this->get_logger(), "🛑 Ensuring motor stays still at startup...");
    
    // 1. Set target velocity to 0 (important if motor was in velocity mode)
    set_target_velocity(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. Ensure we're in position mode (should already be set, but double-check)
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

