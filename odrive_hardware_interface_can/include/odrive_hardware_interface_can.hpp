// Copyright 2021 Factor Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cmath>
#include <map>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "odrive_can.hpp"
#include "visibility_control.hpp"

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

#define CHECK(status)                                                                      \
  do {                                                                                     \
    int ret = (status);                                                                    \
    if (ret != 0) {                                                                        \
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), libusb_error_name(ret)); \
      return return_type::ERROR;                                                           \
    }                                                                                      \
  } while (0)
namespace odrive_hardware_interface {
class ODriveHardwareInterfaceCAN : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ODriveHardwareInterfaceCAN)

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
   hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
   hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
   hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
   hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
   hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read( const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write( const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string *can_tty_;
  int32_t can_speed_;
  int32_t can_serial_speed;
  std::map<int32_t,int32_t> canid_axis_;
  std::vector<int> can_ids_;
  std::vector<int> axes_;
  std::vector<float> vel_gain_;
  std::vector<float> vel_integrator_gain_;
  std::vector<float> torque_constants_;
  std::vector<bool> enable_watchdogs_;
  std::vector<bool> reverse_control_;
  std::vector<bool> hw_cmd_vel_zero_;
  std::vector<float> hw_vel_prev_value_;
  std::vector<bool> hw_vel_suppress_;
  // state interfaces valriables
  std::vector<double> hw_vbus_voltages_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_axis_errors_;
  std::vector<double> hw_motor_temperatures_;
  std::vector<double> hw_motor_current_;
  // command interface variables
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_sent_count_;
  std::vector<double> hw_received_count_;
  // shared with can interface thread
  struct atomic_variables hw_atomics_;
  // thread pointer
  std::thread *can_thread_ptr_;
  enum class integration_level_t : int32_t
  {
    UNDEFINED = 0,
    EFFORT = 1,
    VELOCITY = 2,
    POSITION = 3
  };
  odrive_can odrive_can_;
  std::vector<integration_level_t> control_level_;
};
}  // namespace odrive_hardware_interface_can
