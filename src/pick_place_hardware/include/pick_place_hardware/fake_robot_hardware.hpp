#ifndef PICK_PLACE_HARDWARE__FAKE_ROBOT_HARDWARE_HPP_
#define PICK_PLACE_HARDWARE__FAKE_ROBOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <random>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pick_place_hardware
{
class FakeRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FakeRobotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  double noise_level_;
  double actuator_delay_;  // in seconds
  double position_gain_;   // for first-order lag simulation

  // State storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_target_positions_;  // delayed commands

  // Delay buffer
  std::deque<std::pair<rclcpp::Time, std::vector<double>>> command_buffer_;

  // Random number generator for noise
  std::default_random_engine rng_;
  std::normal_distribution<double> noise_dist_;
};

}  // namespace pick_place_hardware

#endif  // PICK_PLACE_HARDWARE__FAKE_ROBOT_HARDWARE_HPP_
