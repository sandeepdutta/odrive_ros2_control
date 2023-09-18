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
#include <chrono>
#include <mutex>
#include <thread>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "odrive.hpp"
#include "visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

#define CHECK(status)                                                                      \
  do {                                                                                     \
    int ret = (status);                                                                    \
    if (ret != 0) {                                                                        \
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), libusb_error_name(ret)); \
      return return_type::ERROR;                                                           \
    }                                                                                      \
  } while (0)



namespace odrive_hardware_interface
{
class ODriveHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ODriveHardwareInterface)

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
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ODRIVE_HARDWARE_INTERFACE_LOCAL
  hardware_interface::return_type read_into_rt_interface();

  ODRIVE_HARDWARE_INTERFACE_LOCAL
  hardware_interface::return_type write_from_rt_interface();

  ODRIVE_HARDWARE_INTERFACE_LOCAL void usb_read_write_func();
private:

  std::vector<std::vector<int64_t>> serial_numbers_;
  std::vector<int> axes_;
  std::vector<float> torque_constants_;
  std::vector<bool> enable_watchdogs_;
  std::vector<bool> reverse_control_;

  std::vector<double> hw_vbus_voltages_;

  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  std::vector<double> hw_axis_errors_;
  std::vector<double> hw_motor_errors_;
  std::vector<double> hw_encoder_errors_;
  std::vector<double> hw_controller_errors_;
  std::vector<double> hw_fet_temperatures_;
  std::vector<double> hw_motor_temperatures_;
  std::vector<double> hw_motor_current_;
  bool _thread_terminate;
  int  _thread_update_rate_hz;
  std::thread usb_read_write_thread;

  struct _hw_realtime_interface_ {
    bool new_data_avail;
    std::vector<double> hw_vbus_voltages_;

    std::vector<double> hw_commands_positions_;
    std::vector<double> hw_commands_velocities_;
    std::vector<double> hw_commands_efforts_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;

    std::vector<double> hw_axis_errors_;
    std::vector<double> hw_motor_errors_;
    std::vector<double> hw_encoder_errors_;
    std::vector<double> hw_controller_errors_;
    std::vector<double> hw_fet_temperatures_;
    std::vector<double> hw_motor_temperatures_;
    std::vector<double> hw_motor_current_;
  } hw_rt_interface;

  std::mutex rt_mutex_; // mutex to handle realtime transfers

  enum class integration_level_t : int32_t
  {
    UNDEFINED = 0,
    EFFORT = 1,
    VELOCITY = 2,
    POSITION = 3
  };
  std::map<uint64_t, odrive *>  odrives;
  std::vector<integration_level_t> control_level_;
};
}  // namespace odrive_hardware_interface
