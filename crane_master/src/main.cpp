#include "crane_master/canopen_ros2_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CANopenROS2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

