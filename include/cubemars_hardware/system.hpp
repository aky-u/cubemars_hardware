// credit: https://github.com/ros-controls/ros2_control_demos
#ifndef CUBEMARS_HARDWARE__SYSTEM_HPP_
#define CUBEMARS_HARDWARE__SYSTEM_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "cubemars_hardware/can.hpp"
#include "cubemars_hardware/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace cubemars_hardware
{
class CubeMarsSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CubeMarsSystemHardware);

  virtual ~CubeMarsSystemHardware();

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  CUBEMARS_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CUBEMARS_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::return_type
  prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                              const std::vector<std::string> &stop_interfaces) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::return_type
  perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                              const std::vector<std::string> &stop_interfaces) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

private:
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_efforts_;
  std::vector<double> hw_states_temperatures_;

  std::vector<double> erpm_conversions_;
  std::vector<double> torque_constants_;
  std::vector<double> enc_offs_;
  std::vector<double> trq_limits_;
  std::vector<std::pair<std::int16_t, std::int16_t>> limits_;
  std::vector<std::pair<double, double>> position_limits_; // [min, max] If set, ignore any
                                                           // commands to actuate out of range
  std::vector<bool> read_only_;

  CanSocket can_;
  std::string can_itf_;
  std::vector<std::uint32_t> can_ids_;

  enum control_mode_t : std::uint8_t
  {
    CURRENT_LOOP = 1,
    SPEED_LOOP = 3,
    POSITION_LOOP = 4,
    POSITION_SPEED_LOOP = 6,
    UNDEFINED
  };

  // command mode switch variables
  std::vector<bool> stop_modes_;
  std::vector<control_mode_t> start_modes_;

  // active control mode for each actuator
  std::vector<control_mode_t> control_mode_;

  /**
   * @brief Filter command to be within limits to avoid out of range commands.
   * @note Command in the direction towards the limit is allowed.
   */
  std::int32_t filter_command_direction(std::int32_t command, std::pair<double, double> pos_limit,
                                        double current_pos, double enc_offset,
                                        const control_mode_t control_mode);
};

} // namespace cubemars_hardware

#endif // CUBEMARS_HARDWARE__SYSTEM_HPP_
