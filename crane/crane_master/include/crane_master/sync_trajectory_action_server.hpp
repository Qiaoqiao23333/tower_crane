#ifndef SYNC_TRAJECTORY_ACTION_SERVER_HPP
#define SYNC_TRAJECTORY_ACTION_SERVER_HPP

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

#define COB_SYNC 0x080

class SyncTrajectoryActionServer : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    SyncTrajectoryActionServer();
    ~SyncTrajectoryActionServer();

private:
    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle);
    
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    
    void execute(const std::shared_ptr<GoalHandleFJT> goal_handle);

    // CAN communication
    void init_can_socket();
    void send_sync_frame();

    // Member variables
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    
    // Publishers for each motor
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slewing_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_trolley_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hook_;
    
    // Subscribers for current positions
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_slewing_pos_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_trolley_pos_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hook_pos_;
    
    // Current positions
    std::map<std::string, double> current_positions_;
    
    // CAN socket
    std::string can_interface_;
    int can_socket_;
};

#endif // SYNC_TRAJECTORY_ACTION_SERVER_HPP
