#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "std_srvs/srv/trigger.hpp"

// Motor names matching bus.yml configuration
// Note: These should match the device names in bus.yml
// Current bus.yml uses: hook_joint (node_id: 1), trolley_joint (node_id: 2), slewing_joint (node_id: 3)
const std::vector<std::string> MOTOR_NAMES = {"hook_joint", "trolley_joint", "slewing_joint"};

void control_motor(
  rclcpp::Node::SharedPtr node,
  const std::string& motor_name)
{
  RCLCPP_INFO(node->get_logger(), "Controlling motor: %s", motor_name.c_str());

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr init_client =
    node->create_client<std_srvs::srv::Trigger>("/" + motor_name + "/init");

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mode_client =
    node->create_client<std_srvs::srv::Trigger>("/" + motor_name + "/cyclic_position_mode");

  rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedPtr target_client =
    node->create_client<canopen_interfaces::srv::COTargetDouble>("/" + motor_name + "/target");

  // Wait for services to be available
  while (!init_client->wait_for_service(std::chrono::seconds(1)) ||
         !mode_client->wait_for_service(std::chrono::seconds(1)) ||
         !target_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for services. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Services for %s not available, waiting again...", motor_name.c_str());
  }

  // Call init service
  auto trigger_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = init_client->async_send_request(trigger_req);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "[%s] Init service called successfully", motor_name.c_str());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "[%s] Failed to call init service", motor_name.c_str());
    return;
  }

  // Call cyclic_position_mode service
  result = mode_client->async_send_request(trigger_req);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "[%s] Config position mode service called successfully", motor_name.c_str());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "[%s] Failed to call config service", motor_name.c_str());
    return;
  }

  // Start sending target values
  RCLCPP_INFO(node->get_logger(), "[%s] Starting to send target values", motor_name.c_str());
  auto target_req = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
  double target = 0.0;

  while (rclcpp::ok())
  {
    target_req->target = target;

    auto res = target_client->async_send_request(target_req);

    if (rclcpp::spin_until_future_complete(node, res, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "[%s] Set Target: %.2f", motor_name.c_str(), target);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "[%s] Failed to call target service", motor_name.c_str());
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
    target += 1.0;

    if (target >= 105.0)
    {
      target = 0.0;
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("position_tick_motor_node");

  RCLCPP_INFO(node->get_logger(), "Position Tick Motor Node Started - Controlling 3 motors");

  // Create executor for spinning
  rclcpp::executors::MultiThreadedExecutor executor;

  // Create threads for each motor to control them in parallel
  std::vector<std::thread> motor_threads;

  for (const auto& motor_name : MOTOR_NAMES)
  {
    motor_threads.emplace_back([node, motor_name]() {
      control_motor(node, motor_name);
    });
  }

  // Spin the node in a separate thread
  std::thread spin_thread([&executor, node]() {
    executor.add_node(node);
    executor.spin();
  });

  // Wait for all threads to complete (they run until rclcpp::ok() is false)
  for (auto& thread : motor_threads)
  {
    thread.join();
  }

  executor.cancel();
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}

