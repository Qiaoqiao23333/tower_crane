/**
 * @file main.cpp
 * @brief 🏗️ Tower crane CANopen ROS2 control node main entry
 * @details Create and run CANopenROS2 node
 */

#include "crane_master/canopen_ros2_node.hpp"

int main(int argc, char * argv[])
{
    // 🚀 Initialize ROS2
    rclcpp::init(argc, argv);
    
    // 🏗️ Create tower crane control node
    auto node = std::make_shared<CANopenROS2>();
    
    // 🔄 Enter spin loop
    rclcpp::spin(node);
    
    // 🚫 Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}

