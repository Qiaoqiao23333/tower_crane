#include "crane_master/sync_trajectory_action_server.hpp"
#include <fcntl.h>
#include <cstring>

SyncTrajectoryActionServer::SyncTrajectoryActionServer()
    : Node("sync_trajectory_action_server"),
      can_socket_(-1)
{
    // Declare and get parameters
    this->declare_parameter<std::string>("can_interface", "can0");
    can_interface_ = this->get_parameter("can_interface").as_string();
    
    RCLCPP_INFO(this->get_logger(), 
        "🔄 Initializing sync trajectory Action Server, CAN interface: %s", 
        can_interface_.c_str());
    
    // Initialize CAN socket for SYNC frames
    init_can_socket();
    
    // Initialize current positions
    current_positions_["slewing_joint"] = 0.0;
    current_positions_["trolley_joint"] = 0.0;
    current_positions_["hook_joint"] = 0.0;
    
    // Create publishers to send commands to individual motors
    pub_slewing_ = this->create_publisher<std_msgs::msg::Float32>(
        "/slewing/target_position", 10);
    pub_trolley_ = this->create_publisher<std_msgs::msg::Float32>(
        "/trolley/target_position", 10);
    pub_hook_ = this->create_publisher<std_msgs::msg::Float32>(
        "/hoist/target_position", 10);
    
    // Create subscribers to monitor current positions
    sub_slewing_pos_ = this->create_subscription<std_msgs::msg::Float32>(
        "/slewing/crane_position", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            current_positions_["slewing_joint"] = msg->data;
        });
    
    sub_trolley_pos_ = this->create_subscription<std_msgs::msg::Float32>(
        "/trolley/crane_position", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            current_positions_["trolley_joint"] = msg->data;
        });
    
    sub_hook_pos_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hoist/crane_position", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            current_positions_["hook_joint"] = msg->data;
        });
    
    // Create action server
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this,
        "/forward_position_controller/follow_joint_trajectory",
        std::bind(&SyncTrajectoryActionServer::handle_goal, this, 
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&SyncTrajectoryActionServer::handle_cancel, this, 
                  std::placeholders::_1),
        std::bind(&SyncTrajectoryActionServer::handle_accepted, this, 
                  std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), 
        "✅ Sync trajectory Action Server ready: /forward_position_controller/follow_joint_trajectory");
}

SyncTrajectoryActionServer::~SyncTrajectoryActionServer()
{
    if (can_socket_ >= 0) {
        close(can_socket_);
    }
}

void SyncTrajectoryActionServer::init_can_socket()
{
    // Create socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), 
            "❌ Failed to create CAN socket: %s", strerror(errno));
        return;
    }
    
    // Set socket to non-blocking mode
    int flags = fcntl(can_socket_, F_GETFL, 0);
    fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
    
    // Get interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), 
            "❌ Failed to get interface %s index: %s", 
            can_interface_.c_str(), strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    // Bind socket
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), 
            "❌ Failed to bind CAN socket: %s", strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ CAN socket initialized successfully");
}

void SyncTrajectoryActionServer::send_sync_frame()
{
    if (can_socket_ < 0) {
        RCLCPP_WARN(this->get_logger(), "⚠️ CAN socket not initialized, cannot send SYNC");
        return;
    }
    
    struct can_frame frame;
    frame.can_id = COB_SYNC;
    frame.can_dlc = 0;  // SYNC frame has no data
    
    ssize_t bytes_written = write(can_socket_, &frame, sizeof(struct can_frame));
    if (bytes_written != sizeof(struct can_frame)) {
        RCLCPP_WARN(this->get_logger(), 
            "❌ Failed to send SYNC frame: %s", strerror(errno));
    } else {
        RCLCPP_DEBUG(this->get_logger(), "🔄 SYNC frame sent (0x080)");
    }
}

rclcpp_action::GoalResponse SyncTrajectoryActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    (void)uuid;
    
    RCLCPP_INFO(this->get_logger(), 
        "📋 Received trajectory goal with %zu points", 
        goal->trajectory.points.size());
    
    // Validate joint names
    for (const auto& joint_name : goal->trajectory.joint_names) {
        if (joint_name != "slewing_joint" && 
            joint_name != "trolley_joint" && 
            joint_name != "hook_joint") {
            RCLCPP_WARN(this->get_logger(), 
                "⚠️ Trajectory contains unknown joint name: %s", joint_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    
    if (goal->trajectory.points.empty()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Trajectory has no points");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SyncTrajectoryActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "⏹️ Received cancel trajectory request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SyncTrajectoryActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    // Execute in a separate thread to avoid blocking
    std::thread{
        std::bind(&SyncTrajectoryActionServer::execute, this, std::placeholders::_1), 
        goal_handle
    }.detach();
}

void SyncTrajectoryActionServer::execute(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "🎬 Executing sync trajectory...");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    
    auto start_time = std::chrono::steady_clock::now();
    
    // Build joint name to index mapping
    std::map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {
        joint_index_map[goal->trajectory.joint_names[i]] = i;
    }
    
    // Execute each trajectory point
    for (const auto & point : goal->trajectory.points) {
        // Check if goal was cancelled
        if (goal_handle->is_canceling()) {
            result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "❌ Trajectory cancelled");
            return;
        }
        
        // Wait until it's time to execute this point
        auto time_from_start = rclcpp::Duration(point.time_from_start);
        auto target_time = start_time + std::chrono::nanoseconds(time_from_start.nanoseconds());
        auto now = std::chrono::steady_clock::now();
        
        if (target_time > now) {
            std::this_thread::sleep_until(target_time);
        }
        
        // Send position commands to all motors
        std_msgs::msg::Float32 cmd;
        
        // Send commands to each motor based on joint names in trajectory
        if (joint_index_map.find("slewing_joint") != joint_index_map.end()) {
            size_t idx = joint_index_map["slewing_joint"];
            if (idx < point.positions.size()) {
                cmd.data = static_cast<float>(point.positions[idx]);
                pub_slewing_->publish(cmd);
                RCLCPP_DEBUG(this->get_logger(), 
                    "🔄 Slewing command: %.2f degrees", cmd.data);
            }
        }
        
        if (joint_index_map.find("trolley_joint") != joint_index_map.end()) {
            size_t idx = joint_index_map["trolley_joint"];
            if (idx < point.positions.size()) {
                cmd.data = static_cast<float>(point.positions[idx]);
                pub_trolley_->publish(cmd);
                RCLCPP_DEBUG(this->get_logger(), 
                    "🚤 Trolley command: %.2f degrees", cmd.data);
            }
        }
        
        if (joint_index_map.find("hook_joint") != joint_index_map.end()) {
            size_t idx = joint_index_map["hook_joint"];
            if (idx < point.positions.size()) {
                cmd.data = static_cast<float>(point.positions[idx]);
                pub_hook_->publish(cmd);
                RCLCPP_DEBUG(this->get_logger(), 
                    "🎣 Hoist command: %.2f degrees", cmd.data);
            }
        }
        
        // Small delay to ensure commands are sent before SYNC
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        
        // Send SYNC frame to synchronize all motors
        send_sync_frame();
        
        RCLCPP_INFO(this->get_logger(), 
            "📤 Commands sent and synchronized via SYNC frame");
        
        // Publish feedback
        feedback->header.stamp = this->now();
        feedback->joint_names = goal->trajectory.joint_names;
        feedback->actual.positions.clear();
        
        // Fill actual positions from current readings
        for (const auto& joint_name : goal->trajectory.joint_names) {
            feedback->actual.positions.push_back(current_positions_[joint_name]);
        }
        
        feedback->desired.positions = point.positions;
        goal_handle->publish_feedback(feedback);
    }
    
    // Goal succeeded
    if (rclcpp::ok()) {
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), 
            "✅ Sync trajectory execution successful");
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncTrajectoryActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
