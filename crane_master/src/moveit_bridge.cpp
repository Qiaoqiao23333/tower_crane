#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// 辅助函数：角度转弧度
static double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 辅助函数：弧度转角度
static double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

class CraneMoveItBridge : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    CraneMoveItBridge() : Node("moveit_bridge") {
        // -----------------------------------------------------------------------
        // 1. 配置部分 (Configuration)
        // -----------------------------------------------------------------------
        
        // 注意：这里的关节名称顺序必须与 ros2_controllers.yaml 中的配置保持一致
        // 同时也需要与 MoveIt 配置包 (tower_crane_moveit_config) 中的 SRDF/URDF 一致
        joint_names_ = {"joint_hoist", "joint_trolley", "joint_slewing"};

        // -----------------------------------------------------------------------
        // 2. 订阅底层电机反馈 (Subscribers - From CANopen Nodes)
        // -----------------------------------------------------------------------
        // 输入单位：角度 (Degrees)
        sub_hoist_pos_ = this->create_subscription<std_msgs::msg::Float32>(
            "/hoist/crane_position", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { current_positions_deg_["joint_hoist"] = msg->data; });

        sub_trolley_pos_ = this->create_subscription<std_msgs::msg::Float32>(
            "/trolley/crane_position", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { current_positions_deg_["joint_trolley"] = msg->data; });

        sub_slewing_pos_ = this->create_subscription<std_msgs::msg::Float32>(
            "/slewing/crane_position", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { current_positions_deg_["joint_slewing"] = msg->data; });

        // 初始化当前位置为 0
        for (const auto& name : joint_names_) {
            current_positions_deg_[name] = 0.0;
        }

        // -----------------------------------------------------------------------
        // 3. 发布底层电机控制指令 (Publishers - To CANopen Nodes)
        // -----------------------------------------------------------------------
        // 输出单位：角度 (Degrees)
        pub_hoist_cmd_ = this->create_publisher<std_msgs::msg::Float32>("/hoist/target_position", 10);
        pub_trolley_cmd_ = this->create_publisher<std_msgs::msg::Float32>("/trolley/target_position", 10);
        pub_slewing_cmd_ = this->create_publisher<std_msgs::msg::Float32>("/slewing/target_position", 10);

        // -----------------------------------------------------------------------
        // 4. 发布 Joint States 给 MoveIt (Publisher - To MoveIt)
        // -----------------------------------------------------------------------
        // 输出单位：弧度 (Radians)
        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 20Hz 定时器发布关节状态
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&CraneMoveItBridge::publish_joint_states, this));

        // -----------------------------------------------------------------------
        // 5. Action Server (Command Interface from MoveIt)
        // -----------------------------------------------------------------------
        // 注意：Action 名称 "/crane_arm_controller/follow_joint_trajectory" 
        // 必须与 ros2_controllers.yaml 中定义的控制器命名空间匹配。
        // 例如 YAML 中为:
        // crane_arm_controller:
        //   ros__parameters:
        //     ...
        // 则 Action Server 应为 /crane_arm_controller/follow_joint_trajectory
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "/crane_arm_controller/follow_joint_trajectory",
            std::bind(&CraneMoveItBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CraneMoveItBridge::handle_cancel, this, std::placeholders::_1),
            std::bind(&CraneMoveItBridge::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "CraneMoveItBridge initialized.");
    }

private:
    // 数据成员
    std::vector<std::string> joint_names_;
    std::map<std::string, double> current_positions_deg_; // 存储最新的电机位置(角度)

    // ROS 接口
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hoist_pos_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_trolley_pos_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_slewing_pos_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hoist_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_trolley_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slewing_cmd_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    // -----------------------------------------------------------------------
    // 关节状态发布逻辑
    // -----------------------------------------------------------------------
    void publish_joint_states() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        // 将当前存储的角度转换为弧度发布
        for (const auto& name : joint_names_) {
            msg.position.push_back(deg2rad(current_positions_deg_[name]));
        }

        pub_joint_states_->publish(msg);
    }

    // -----------------------------------------------------------------------
    // Action Server 回调
    // -----------------------------------------------------------------------
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request with %zu trajectory points", goal->trajectory.points.size());
        
        // 简单的验证：检查关节名称是否匹配
        for (const auto& name : goal->trajectory.joint_names) {
            bool found = false;
            for (const auto& my_joint : joint_names_) {
                if (name == my_joint) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                RCLCPP_WARN(this->get_logger(), "Unknown joint in goal: %s", name.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        // 在新线程中执行，避免阻塞
        std::thread{std::bind(&CraneMoveItBridge::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // -----------------------------------------------------------------------
    // 执行逻辑
    // -----------------------------------------------------------------------
    void execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        auto result = std::make_shared<FollowJointTrajectory::Result>();

        auto start_time = std::chrono::steady_clock::now();

        // 建立目标关节名称到 trajectory 索引的映射
        // MoveIt 发送的 joint_names 顺序可能与我们的 joint_names_ 不一致
        std::vector<int> map_traj_to_local(goal->trajectory.joint_names.size(), -1);
        for (size_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {
            std::string name = goal->trajectory.joint_names[i];
            if (name == "joint_hoist") map_traj_to_local[i] = 0; // 0 对应 hoist
            else if (name == "joint_trolley") map_traj_to_local[i] = 1; // 1 对应 trolley
            else if (name == "joint_slewing") map_traj_to_local[i] = 2; // 2 对应 slewing
        }

        for (const auto & point : goal->trajectory.points) {
            if (goal_handle->is_canceling()) {
                result->error_code = control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // 1. 时间同步
            // 读取该点应该在何时执行 (time_from_start)
            auto time_from_start = point.time_from_start;
            auto duration_ns = rclcpp::Duration(time_from_start).nanoseconds();
            
            auto now = std::chrono::steady_clock::now();
            auto target_time = start_time + std::chrono::nanoseconds(duration_ns);
            
            if (target_time > now) {
                std::this_thread::sleep_until(target_time);
            }

            // 2. 发送控制指令
            // 遍历轨迹点中的每个关节位置
            for (size_t i = 0; i < point.positions.size(); ++i) {
                int type = map_traj_to_local[i];
                double pos_rad = point.positions[i];
                double pos_deg = rad2deg(pos_rad);
                
                std_msgs::msg::Float32 msg;
                msg.data = static_cast<float>(pos_deg);

                if (type == 0) { // Hoist
                    pub_hoist_cmd_->publish(msg);
                } else if (type == 1) { // Trolley
                    pub_trolley_cmd_->publish(msg);
                } else if (type == 2) { // Slewing
                    pub_slewing_cmd_->publish(msg);
                }
            }

            // 3. 反馈 (可选，简单起见这里只更新 header)
            feedback->header.stamp = this->now();
            feedback->joint_names = goal->trajectory.joint_names;
            feedback->actual.positions = point.positions; // 这里实际上应该填当前真实值
            goal_handle->publish_feedback(feedback);
        }

        // 执行完毕
        if (rclcpp::ok()) {
            result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CraneMoveItBridge>());
    rclcpp::shutdown();
    return 0;
}

