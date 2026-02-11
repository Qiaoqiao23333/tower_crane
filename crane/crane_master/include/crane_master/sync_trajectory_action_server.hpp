#ifndef SYNC_TRAJECTORY_ACTION_SERVER_HPP
#define SYNC_TRAJECTORY_ACTION_SERVER_HPP

/**
 * @file sync_trajectory_action_server.hpp
 * @brief 🔄 同步轨迹 Action Server 头文件
 * @details 实现多电机同步轨迹控制，支持 MoveIt 的 FollowJointTrajectory Action
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

#define COB_SYNC 0x080  // 🔄 CANopen 同步帧 COB-ID

/**
 * @class SyncTrajectoryActionServer
 * @brief 🔄 同步轨迹 Action Server 类
 * @details 接收 MoveIt 的轨迹指令，同步控制多个电机运动
 */
class SyncTrajectoryActionServer : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    SyncTrajectoryActionServer();   // 🛠️ 构造函数
    ~SyncTrajectoryActionServer();  // 🗑️ 析构函数

private:
    // ==================== 🎯 Action Server 回调函数 ====================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);  // 📋 处理目标请求
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle);         // ❌ 处理取消请求
    
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);  // ✅ 处理接受的目标
    
    void execute(const std::shared_ptr<GoalHandleFJT> goal_handle);          // 🎬 执行轨迹

    // ==================== 📡 CAN 通信函数 ====================
    void init_can_socket();   // 🔌 初始化 CAN 套接字
    void send_sync_frame();   // 🔄 发送同步帧

    // ==================== 📦 成员变量 ====================
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;  // 🎯 Action 服务器
    
    // 📤 各电机的位置命令发布器
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slewing_;   // 🔄 回转电机
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_trolley_;   // 🚤 小车电机
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hook_;      // 🎣 起升电机
    
    // 📥 各电机的当前位置订阅器
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_slewing_pos_;   // 🔄 回转位置
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_trolley_pos_;   // 🚤 小车位置
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hook_pos_;      // 🎣 升降位置
    
    // 📍 当前位置存储
    std::map<std::string, double> current_positions_;
    
    // 📡 CAN 套接字
    std::string can_interface_;   // 📟 CAN 接口名称
    int can_socket_;              // 🔌 套接字文件描述符
};

#endif // SYNC_TRAJECTORY_ACTION_SERVER_HPP
