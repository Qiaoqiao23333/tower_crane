/**
 * @file main.cpp
 * @brief 🏗️ 塔吊 CANopen ROS2 控制节点主入口
 * @details 创建并运行 CANopenROS2 节点
 */

#include "crane_master/canopen_ros2_node.hpp"

int main(int argc, char * argv[])
{
    // 🚀 初始化 ROS2
    rclcpp::init(argc, argv);
    
    // 🏗️ 创建塔吊控制节点
    auto node = std::make_shared<CANopenROS2>();
    
    // 🔄 进入 spin 循环
    rclcpp::spin(node);
    
    // 🚫 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}

