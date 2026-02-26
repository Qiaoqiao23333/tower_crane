#ifndef SYNC_TRAJECTORY_ACTION_SERVER_HPP
#define SYNC_TRAJECTORY_ACTION_SERVER_HPP

/**
 * @file sync_trajectory_action_server.hpp
 * @brief 🔄 Sync trajectory Action Server header file
 * @details Implements multi-motor synchronized trajectory control, supports MoveIt's FollowJointTrajectory Action
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/float32.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <chrono>

#define COB_SYNC 0x080  // 🔄 CANopen sync frame COB-ID

/**
 * @class SyncTrajectoryActionServer
 * @brief 🔄 Sync trajectory Action Server class
 * @details Receives MoveIt trajectory commands, synchronously controls multiple motors
 */
class SyncTrajectoryActionServer : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    SyncTrajectoryActionServer();   // 🛠️ Constructor
    ~SyncTrajectoryActionServer();  // 🗑️ Destructor

private:
    // ==================== 🎯 Action Server callback functions ====================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);  // 📋 Handle goal request
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle);         // ❌ Handle cancel request
    
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);  // ✅ Handle accepted goal
    
    void execute(const std::shared_ptr<GoalHandleFJT> goal_handle);          // 🎬 Execute trajectory

    // ==================== 📡 CAN communication functions ====================
    void init_can_socket();   // 🔌 Initialize CAN socket
    void send_sync_frame();   // 🔄 Send sync frame

    // ==================== 📦 Member variables ====================
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;  // 🎯 Action server
    
    // 📤 Position command publishers for each motor
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slewing_;   // 🔄 Slewing motor
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_trolley_;   // 🚤 Trolley motor
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hook_;      // 🎣 Hoist motor
    
    // 📥 Current position subscribers for each motor
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_slewing_pos_;   // 🔄 Slewing position
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_trolley_pos_;   // 🚤 Trolley position
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hook_pos_;      // 🎣 Hoist position
    
    // 📍 Current position storage
    std::map<std::string, double> current_positions_;
    
    // 📡 CAN socket
    std::string can_interface_;   // 📟 CAN interface name
    int can_socket_;              // 🔌 Socket file descriptor
};

#endif // SYNC_TRAJECTORY_ACTION_SERVER_HPP
