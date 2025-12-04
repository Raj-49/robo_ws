#include "pick_place_hardware/fake_robot_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pick_place_hardware
{
hardware_interface::CallbackReturn FakeRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters
  noise_level_ = 0.001;  // default 1mm noise
  actuator_delay_ = 0.05;  // default 50ms delay
  position_gain_ = 5.0;  // default gain for lag simulation

  if (info_.hardware_parameters.find("noise_level") != info_.hardware_parameters.end()) {
    noise_level_ = std::stod(info_.hardware_parameters["noise_level"]);
  }
  if (info_.hardware_parameters.find("actuator_delay") != info_.hardware_parameters.end()) {
    actuator_delay_ = std::stod(info_.hardware_parameters["actuator_delay"]);
  }
  if (info_.hardware_parameters.find("position_gain") != info_.hardware_parameters.end()) {
    position_gain_ = std::stod(info_.hardware_parameters["position_gain"]);
  }

  // Initialize state vectors
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_target_positions_.resize(info_.joints.size(), 0.0);

  // Initialize random number generator
  rng_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  noise_dist_ = std::normal_distribution<double>(0.0, noise_level_);

  RCLCPP_INFO(
    rclcpp::get_logger("FakeRobotHardware"),
    "Initialized FakeRobotHardware with %zu joints, noise=%.4f, delay=%.3fs",
    info_.joints.size(), noise_level_, actuator_delay_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeRobotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FakeRobotHardware"), "Configuring...");
  
  // Reset all states to zero
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_target_positions_.begin(), hw_target_positions_.end(), 0.0);
  command_buffer_.clear();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FakeRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FakeRobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FakeRobotHardware"), "Activating...");
  
  // Initialize commands to current positions
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = hw_positions_[i];
    hw_target_positions_[i] = hw_positions_[i];
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FakeRobotHardware"), "Deactivating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FakeRobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Simulate reading from hardware
  // Add noise to positions (simulating encoder noise)
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    // Positions are already updated in write(), just add noise for realism
    // (In real hardware, read() would actually query the device)
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FakeRobotHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Add current command to delay buffer
  command_buffer_.push_back({time, hw_commands_});

  // Remove old commands from buffer (older than delay time)
  while (!command_buffer_.empty()) {
    auto age = (time - command_buffer_.front().first).seconds();
    if (age >= actuator_delay_) {
      hw_target_positions_ = command_buffer_.front().second;
      command_buffer_.pop_front();
      break;
    } else {
      break;  // Buffer is sorted by time, so we can stop
    }
  }

  // If buffer is empty (startup), use current command
  if (command_buffer_.empty()) {
    hw_target_positions_ = hw_commands_;
  }

  // Simulate actuator dynamics with first-order lag
  double dt = period.seconds();
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    // First-order lag: pos += (target - pos) * gain * dt
    double error = hw_target_positions_[i] - hw_positions_[i];
    double delta_pos = error * position_gain_ * dt;
    
    hw_positions_[i] += delta_pos;
    hw_velocities_[i] = delta_pos / dt;  // Approximate velocity
    
    // Add small noise to position
    hw_positions_[i] += noise_dist_(rng_);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace pick_place_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pick_place_hardware::FakeRobotHardware, hardware_interface::SystemInterface)
