#ifndef TOWER_CRANE_HARDWARE__TOWER_CRANE_TRANSMISSION_SYSTEM_HPP_
#define TOWER_CRANE_HARDWARE__TOWER_CRANE_TRANSMISSION_SYSTEM_HPP_

#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace tower_crane_hardware
{

class TowerCraneTransmissionSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TowerCraneTransmissionSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  double slewing_reduction_ = 1.0;
  double trolley_reduction_ = 1.0;
  double hook_reduction_ = 1.0;

  double slewing_pos_ = 0.0;
  double trolley_pos_ = 0.0;
  double hook_pos_ = 0.0;
  double slewing_vel_ = 0.0;
  double trolley_vel_ = 0.0;
  double hook_vel_ = 0.0;
  double slewing_cmd_ = 0.0;
  double trolley_cmd_ = 0.0;
  double hook_cmd_ = 0.0;

  double slewing_motor_pos_ = 0.0;
  double trolley_motor_pos_ = 0.0;
  double hook_motor_pos_ = 0.0;
  double slewing_motor_vel_ = 0.0;
  double trolley_motor_vel_ = 0.0;
  double hook_motor_vel_ = 0.0;
  double slewing_motor_cmd_ = 0.0;
  double trolley_motor_cmd_ = 0.0;
  double hook_motor_cmd_ = 0.0;
};

}  // namespace tower_crane_hardware

#endif
