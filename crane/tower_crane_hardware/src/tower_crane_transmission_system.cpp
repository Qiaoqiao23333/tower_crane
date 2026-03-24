#include "tower_crane_hardware/tower_crane_transmission_system.hpp"

#include <cstddef>
#include <cstdlib>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tower_crane_hardware
{

namespace
{

double get_reduction(const hardware_interface::ComponentInfo & joint)
{
  const auto it = joint.parameters.find("mechanical_reduction");
  if (it == joint.parameters.end()) {
    return 1.0;
  }

  const double reduction = std::strtod(it->second.c_str(), nullptr);
  return reduction == 0.0 ? 1.0 : reduction;
}

}  // namespace

hardware_interface::CallbackReturn TowerCraneTransmissionSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  constexpr std::size_t expected_joint_count = 3;
  if (info_.joints.size() != expected_joint_count) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TowerCraneTransmissionSystem"),
      "Expected %zu joints in ros2_control config, got %zu",
      expected_joint_count, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1U ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("TowerCraneTransmissionSystem"),
        "Joint %s must expose exactly one position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.empty() ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("TowerCraneTransmissionSystem"),
        "Joint %s must expose a position state interface as its first state",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const double reduction = get_reduction(joint);
    if (joint.name == "slewing_joint") {
      slewing_reduction_ = reduction;
    } else if (joint.name == "trolley_joint") {
      trolley_reduction_ = reduction;
    } else if (joint.name == "hook_joint") {
      hook_reduction_ = reduction;
    }
  }

  slewing_pos_ = trolley_pos_ = hook_pos_ = 0.0;
  slewing_vel_ = trolley_vel_ = hook_vel_ = 0.0;
  slewing_cmd_ = trolley_cmd_ = hook_cmd_ = 0.0;
  slewing_motor_pos_ = trolley_motor_pos_ = hook_motor_pos_ = 0.0;
  slewing_motor_vel_ = trolley_motor_vel_ = hook_motor_vel_ = 0.0;
  slewing_motor_cmd_ = trolley_motor_cmd_ = hook_motor_cmd_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TowerCraneTransmissionSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("slewing_joint", hardware_interface::HW_IF_POSITION, &slewing_pos_);
  state_interfaces.emplace_back("slewing_joint", hardware_interface::HW_IF_VELOCITY, &slewing_vel_);
  state_interfaces.emplace_back("trolley_joint", hardware_interface::HW_IF_POSITION, &trolley_pos_);
  state_interfaces.emplace_back("trolley_joint", hardware_interface::HW_IF_VELOCITY, &trolley_vel_);
  state_interfaces.emplace_back("hook_joint", hardware_interface::HW_IF_POSITION, &hook_pos_);
  state_interfaces.emplace_back("hook_joint", hardware_interface::HW_IF_VELOCITY, &hook_vel_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TowerCraneTransmissionSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    "slewing_joint", hardware_interface::HW_IF_POSITION, &slewing_cmd_);
  command_interfaces.emplace_back(
    "trolley_joint", hardware_interface::HW_IF_POSITION, &trolley_cmd_);
  command_interfaces.emplace_back("hook_joint", hardware_interface::HW_IF_POSITION, &hook_cmd_);
  return command_interfaces;
}

hardware_interface::CallbackReturn TowerCraneTransmissionSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  slewing_cmd_ = slewing_pos_;
  trolley_cmd_ = trolley_pos_;
  hook_cmd_ = hook_pos_;
  slewing_vel_ = trolley_vel_ = hook_vel_ = 0.0;
  slewing_motor_cmd_ = slewing_cmd_ * slewing_reduction_;
  trolley_motor_cmd_ = trolley_cmd_ * trolley_reduction_;
  hook_motor_cmd_ = hook_cmd_ * hook_reduction_;
  slewing_motor_pos_ = slewing_pos_ * slewing_reduction_;
  trolley_motor_pos_ = trolley_pos_ * trolley_reduction_;
  hook_motor_pos_ = hook_pos_ * hook_reduction_;
  slewing_motor_vel_ = trolley_motor_vel_ = hook_motor_vel_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TowerCraneTransmissionSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  slewing_vel_ = trolley_vel_ = hook_vel_ = 0.0;
  slewing_motor_vel_ = trolley_motor_vel_ = hook_motor_vel_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TowerCraneTransmissionSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  slewing_pos_ = slewing_motor_pos_ / slewing_reduction_;
  trolley_pos_ = trolley_motor_pos_ / trolley_reduction_;
  hook_pos_ = hook_motor_pos_ / hook_reduction_;
  slewing_vel_ = slewing_motor_vel_ / slewing_reduction_;
  trolley_vel_ = trolley_motor_vel_ / trolley_reduction_;
  hook_vel_ = hook_motor_vel_ / hook_reduction_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TowerCraneTransmissionSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  slewing_motor_cmd_ = slewing_cmd_ * slewing_reduction_;
  trolley_motor_cmd_ = trolley_cmd_ * trolley_reduction_;
  hook_motor_cmd_ = hook_cmd_ * hook_reduction_;

  if (dt > 0.0) {
    slewing_motor_vel_ = (slewing_motor_cmd_ - slewing_motor_pos_) / dt;
    trolley_motor_vel_ = (trolley_motor_cmd_ - trolley_motor_pos_) / dt;
    hook_motor_vel_ = (hook_motor_cmd_ - hook_motor_pos_) / dt;
  } else {
    slewing_motor_vel_ = trolley_motor_vel_ = hook_motor_vel_ = 0.0;
  }

  // Mirror the motor command into feedback until the real actuator transport is connected.
  slewing_motor_pos_ = slewing_motor_cmd_;
  trolley_motor_pos_ = trolley_motor_cmd_;
  hook_motor_pos_ = hook_motor_cmd_;
  return hardware_interface::return_type::OK;
}

}  // namespace tower_crane_hardware

PLUGINLIB_EXPORT_CLASS(
  tower_crane_hardware::TowerCraneTransmissionSystem,
  hardware_interface::SystemInterface)
