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

// 📐 Helper function: degrees to radians (for revolute joints)
static double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 📐 Helper function: radians to degrees (for revolute joints)
static double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// 📐 Helper function: degrees to meters (for prismatic joints: Hoist, Trolley)
// ⚠️ TODO: User needs to modify this conversion coefficient based on actual hoist/lead screw parameters
// Assumption: linear movement distance (meters) corresponding to motor rotation of 1 degree
static const double METERS_PER_DEGREE_HOIST = 0.001; // Example value
static const double METERS_PER_DEGREE_TROLLEY = 0.001; // Example value

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
        // 1. Configuration Section
        // -----------------------------------------------------------------------
        
        // Note: The joint name order here must match the configuration in ros2_controllers.yaml
        // Also needs to match the SRDF/URDF in the MoveIt configuration package (tower_crane_moveit_config)
        joint_names_ = {"hook_joint", "trolley_joint", "slewing_joint"};

        // -----------------------------------------------------------------------
        // 2. Subscribe to Low-level Motor Feedback (Subscribers - From CANopen Nodes)
        // -----------------------------------------------------------------------
        // Input unit: Degrees
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
        // 3. Publish Low-level Motor Control Commands (Publishers - To CANopen Nodes)
        // -----------------------------------------------------------------------
        // Output unit: Degrees
        pub_hoist_cmd_ = this->create_publisher<std_msgs::msg::Float32>("/hoist/target_position", 10);
        pub_trolley_cmd_ = this->create_publisher<std_msgs::msg::Float32>("/trolley/target_position", 10);
        pub_slewing_cmd_ = this->create_publisher<std_msgs::msg::Float32>("/slewing/target_position", 10);

        // -----------------------------------------------------------------------
        // 4. Publish Joint States to MoveIt (Publisher - To MoveIt)
        // -----------------------------------------------------------------------
        // Output unit: Radians
        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 20Hz timer to publish joint states
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&CraneMoveItBridge::publish_joint_states, this));

        // -----------------------------------------------------------------------
        // 5. Action Server (Command Interface from MoveIt)
        // -----------------------------------------------------------------------
        // Note: Action name "/crane_arm_controller/follow_joint_trajectory" 
        // must match the controller namespace defined in ros2_controllers.yaml.
        // For example, if YAML has:
        // crane_arm_controller:
        //   ros__parameters:
        //     ...
        // Then Action Server should be /crane_arm_controller/follow_joint_trajectory
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "/forward_position_controller/follow_joint_trajectory",
            std::bind(&CraneMoveItBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CraneMoveItBridge::handle_cancel, this, std::placeholders::_1),
            std::bind(&CraneMoveItBridge::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "✅ CraneMoveItBridge initialization complete");
    }

private:
    // Data members
    std::vector<std::string> joint_names_;
    std::map<std::string, double> current_positions_deg_; // Store latest motor positions (degrees)

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
    // Joint State Publishing Logic
    // -----------------------------------------------------------------------
    void publish_joint_states() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        // Convert currently stored angles to units required by MoveIt (radians or meters)
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
    // Action Server Callbacks
    // -----------------------------------------------------------------------
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "📋 Received goal request with %zu trajectory points", goal->trajectory.points.size());
        
        // Simple validation: check if joint names match
        for (const auto& name : goal->trajectory.joint_names) {
            bool found = false;
            for (const auto& my_joint : joint_names_) {
                if (name == my_joint) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Goal contains unknown joint: %s", name.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "⏹️ Received cancel goal request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        // Execute in new thread to avoid blocking
        std::thread{std::bind(&CraneMoveItBridge::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // -----------------------------------------------------------------------
    // Execution Logic
    // -----------------------------------------------------------------------
    void execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "🎬 Executing trajectory...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        auto result = std::make_shared<FollowJointTrajectory::Result>();

        auto start_time = std::chrono::steady_clock::now();

        // Build mapping from goal joint names to trajectory indices
        // The joint_names order sent by MoveIt may not match our joint_names_
        std::vector<int> map_traj_to_local(goal->trajectory.joint_names.size(), -1);
        for (size_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {
            std::string name = goal->trajectory.joint_names[i];
            if (name == "hook_joint") map_traj_to_local[i] = 0; // 0 corresponds to hoist
            else if (name == "trolley_joint") map_traj_to_local[i] = 1; // 1 corresponds to trolley
            else if (name == "slewing_joint") map_traj_to_local[i] = 2; // 2 corresponds to slewing
        }

        for (const auto & point : goal->trajectory.points) {
            if (goal_handle->is_canceling()) {
                result->error_code = control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "😾 Goal canceled");
                return;
            }

            // 1. Time synchronization
            // Read when this point should be executed (time_from_start)
            auto time_from_start = point.time_from_start;
            auto duration_ns = rclcpp::Duration(time_from_start).nanoseconds();
            
            auto now = std::chrono::steady_clock::now();
            auto target_time = start_time + std::chrono::nanoseconds(duration_ns);
            
            if (target_time > now) {
                std::this_thread::sleep_until(target_time);
            }

            // 2. Send control commands
            // Iterate through each joint position in the trajectory point
            for (size_t i = 0; i < point.positions.size(); ++i) {
                int type = map_traj_to_local[i];
                double target_val = point.positions[i]; // Radians (Revolute) or meters (Prismatic)
                double cmd_deg = 0.0;
                
                if (type == 0) { // Hoist (Hook) - Prismatic
                    // Meters -> degrees
                    cmd_deg = meter2deg(target_val, METERS_PER_DEGREE_HOIST);
                } else if (type == 1) { // Trolley - Prismatic
                    // Meters -> degrees
                    cmd_deg = meter2deg(target_val, METERS_PER_DEGREE_TROLLEY);
                } else if (type == 2) { // Slewing - Revolute
                    // Radians -> degrees
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

            // 3. Feedback (optional, for simplicity only update header here)
            feedback->header.stamp = this->now();
            feedback->joint_names = goal->trajectory.joint_names;
            feedback->actual.positions = point.positions; // Should actually fill with current real values here
            goal_handle->publish_feedback(feedback);
        }

        // Execution complete
        if (rclcpp::ok()) {
            result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "😼👏 Goal execution successful");
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

