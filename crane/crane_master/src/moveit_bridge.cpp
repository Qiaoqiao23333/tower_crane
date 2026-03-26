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

// Helper: degrees to radians (for revolute joints)
static double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// Helper: radians to degrees (for revolute joints)
static double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// Helper: degrees to meters (for prismatic joints: Hoist, Trolley)
// NOTE: for the hoist we now treat the numeric value as 1:1 (no scaling) between
//       motor "degrees" and the MoveIt linear unit used for the joint.
//       Adjust here if you later need a physical conversion again.
static const double METERS_PER_DEGREE_HOIST = 1.0;    // hoist: 1:1 mapping
static const double METERS_PER_DEGREE_TROLLEY = 1.0;  // trolley: 1:1 mapping

static double deg2meter(double deg, double ratio) {
    return deg * ratio;
}

static double meter2deg(double meter, double ratio) {
    if (std::abs(ratio) < 1e-9) return 0.0;
    return meter / ratio;
}

class CraneMoveItBridge : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    CraneMoveItBridge() : Node("moveit_bridge") {
        // -----------------------------------------------------------------------
        // 1. Configuration
        // -----------------------------------------------------------------------
        
        // NOTE: joint name ordering must match ros2_controllers.yaml
        // and the SRDF/URDF in the MoveIt config package (tower_crane_moveit_config)
        joint_names_ = {"hook_joint", "trolley_joint", "slewing_joint"};

        // -----------------------------------------------------------------------
        // 2. Subscribers for low-level motor feedback (from CANopen nodes)
        // -----------------------------------------------------------------------
        // Input units: degrees
        sub_hoist_pos_ = this->create_subscription<std_msgs::msg::Float32>(
            "/hoist/crane_position", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { current_positions_deg_["hook_joint"] = msg->data; });

        sub_trolley_pos_ = this->create_subscription<std_msgs::msg::Float32>(
            "/trolley/crane_position", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { current_positions_deg_["trolley_joint"] = msg->data; });

        sub_slewing_pos_ = this->create_subscription<std_msgs::msg::Float32>(
            "/slewing/crane_position", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { current_positions_deg_["slewing_joint"] = msg->data; });

        // Initialize current positions to 0
        for (const auto& name : joint_names_) {
            current_positions_deg_[name] = 0.0;
        }

        // -----------------------------------------------------------------------
        // 3. Publishers for low-level motor commands (to CANopen nodes)
        // -----------------------------------------------------------------------
        // Output units: degrees
        pub_hoist_cmd_ = this->create_publisher<std_msgs::msg::Float32>("/hoist/target_position", 10);
        pub_trolley_cmd_ = this->create_publisher<std_msgs::msg::Float32>("/trolley/target_position", 10);
        pub_slewing_cmd_ = this->create_publisher<std_msgs::msg::Float32>("/slewing/target_position", 10);

        // -----------------------------------------------------------------------
        // 4. JointState publisher for MoveIt
        // -----------------------------------------------------------------------
        // Output units: radians (revolute) or meters (prismatic)
        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 20 Hz timer to publish joint states
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&CraneMoveItBridge::publish_joint_states, this));

        // -----------------------------------------------------------------------
        // 5. Action Server (command interface from MoveIt)
        // -----------------------------------------------------------------------
        // NOTE: action name "/crane_arm_controller/follow_joint_trajectory"
        // must match the controller namespace in ros2_controllers.yaml.
        // For example, if YAML defines:
        // crane_arm_controller:
        //   ros__parameters:
        //     ...
        // then the action server should be /crane_arm_controller/follow_joint_trajectory
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "/forward_position_controller/follow_joint_trajectory",
            std::bind(&CraneMoveItBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CraneMoveItBridge::handle_cancel, this, std::placeholders::_1),
            std::bind(&CraneMoveItBridge::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "CraneMoveItBridge initialized.");
    }

private:
    // Data members
    std::vector<std::string> joint_names_;
    std::map<std::string, double> current_positions_deg_; // latest motor positions in degrees
    
    // ROS interfaces
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
    // Joint state publishing
    // -----------------------------------------------------------------------
    void publish_joint_states() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        // Convert stored degrees to units required by MoveIt (radians or meters)
        for (const auto& name : joint_names_) {
            double raw_deg = current_positions_deg_[name];
            double val_converted = 0.0;

            if (name == "slewing_joint") {
                // Revolute joint: degrees -> radians
                val_converted = deg2rad(raw_deg);
            } else if (name == "hook_joint") {
                // Prismatic joint: degrees -> meters
                val_converted = deg2meter(raw_deg, METERS_PER_DEGREE_HOIST);
            } else if (name == "trolley_joint") {
                // Prismatic joint: degrees -> meters
                val_converted = deg2meter(raw_deg, METERS_PER_DEGREE_TROLLEY);
            }
            
            msg.position.push_back(val_converted);
        }

        pub_joint_states_->publish(msg);
    }

    // -----------------------------------------------------------------------
    // Action server callbacks
    // -----------------------------------------------------------------------
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request with %zu trajectory points", goal->trajectory.points.size());
        
        // Basic validation: ensure joint names match our expectations
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
    // Execution logic
    // -----------------------------------------------------------------------
    void execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        auto result = std::make_shared<FollowJointTrajectory::Result>();

        auto start_time = std::chrono::steady_clock::now();
        
        // Map incoming joint names to our internal joint indices.
        // MoveIt joint order may differ from our joint_names_.
        std::vector<int> map_traj_to_local(goal->trajectory.joint_names.size(), -1);
        for (size_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {
            std::string name = goal->trajectory.joint_names[i];
            if (name == "hook_joint") map_traj_to_local[i] = 0; // 0 → hoist
            else if (name == "trolley_joint") map_traj_to_local[i] = 1; // 1 → trolley
            else if (name == "slewing_joint") map_traj_to_local[i] = 2; // 2 → slewing
        }

        for (const auto & point : goal->trajectory.points) {
            if (goal_handle->is_canceling()) {
                result->error_code = control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // 1. time synchronization
            // read the time should be executed for this point (time_from_start)
            auto time_from_start = point.time_from_start;
            auto duration_ns = rclcpp::Duration(time_from_start).nanoseconds();
            
            auto now = std::chrono::steady_clock::now();
            auto target_time = start_time + std::chrono::nanoseconds(duration_ns);
            
            if (target_time > now) {
                std::this_thread::sleep_until(target_time);
            }
            
            // 2. Send control commands for each joint in this trajectory point
            for (size_t i = 0; i < point.positions.size(); ++i) {
                int type = map_traj_to_local[i];
                double target_val = point.positions[i]; // radians (revolute) or meters (prismatic)
                double cmd_deg = 0.0;
                
                if (type == 0) { // Hoist (hook) - prismatic
                    // meters -> degrees
                    cmd_deg = meter2deg(target_val, METERS_PER_DEGREE_HOIST);
                } else if (type == 1) { // Trolley - prismatic
                    // meters -> degrees
                    cmd_deg = meter2deg(target_val, METERS_PER_DEGREE_TROLLEY);
                } else if (type == 2) { // Slewing - revolute
                    // MoveIt uses radians for revolute joints, but the motor API expects degrees.
                    cmd_deg = rad2deg(target_val);
                }

                std_msgs::msg::Float32 msg;
                msg.data = static_cast<float>(cmd_deg);

                if (type == 0) { // Hoist
                    pub_hoist_cmd_->publish(msg);
                } else if (type == 1) { // Trolley
                    pub_trolley_cmd_->publish(msg);
                } else if (type == 2) { // Slewing
                    pub_slewing_cmd_->publish(msg);
                }
            }

            // 3. Feedback (simple: update header and echo desired positions)
            feedback->header.stamp = this->now();
            feedback->joint_names = goal->trajectory.joint_names;
            feedback->actual.positions = point.positions; // here should be the actual values
            goal_handle->publish_feedback(feedback);
        }

        // finished
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

