
#include "odrive_hardware_interface_can.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odrive_hardware_interface
{
hardware_interface::CallbackReturn ODriveHardwareInterfaceCAN::on_init(const hardware_interface::HardwareInfo & info)
{
  if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return  hardware_interface::CallbackReturn::ERROR;
  }

  can_ids_.resize(info_.joints.size());
  torque_constants_.resize(info_.joints.size());
  reverse_control_.resize(info_.joints.size(), false);
  enable_watchdogs_.resize(info_.joints.size());
  hw_atomics_ = new struct atomic_variables();

  // state interfaces
  hw_vbus_voltages_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_axis_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_current_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // command interfaces
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  /* odrive global parameters */
  for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    can_tty_ = new std::string(sensor.parameters.at("can_tty"));
    can_speed_    = std::stoi(sensor.parameters.at("can_speed"));
  }
  /* axis related parameters */
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    can_ids_.emplace_back(std::stoull(joint.parameters.at("can_id"), 0, 16));
    axes_.emplace_back(std::stoi(joint.parameters.at("axis")));
  }
  // create the canid <--> axis map
  for (size_t i = 0 ; i < info_.joints.size(); i++) {
    canid_axis_[can_ids_[i]] = axes_[i]; 
  }
  odrive_can_ = new odrive_can(can_tty_, can_speed_, hw_atomics_, &canid_axis_); // open and initialize the can <-> serila interface
  // start the odrive thread
  can_thread_ptr_ = std::unique_ptr<std::thread>(new std::thread(std::bind(&odrive_can::can_thread,*odrive_can_)));

  ROS_INFO("Configured");
  return  hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterfaceCAN::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[i].name, "vbus_voltage", &hw_vbus_voltages_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "axis_error", &hw_axis_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "motor_temperature", &hw_motor_temperatures_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "motor_current", &hw_motor_temperatures_[i]));
  }
  ROS_INFO("States exported ..");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterfaceCAN::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }
  ROS_INFO("Commands exported ..");
  return command_interfaces;
}

 hardware_interface::return_type ODriveHardwareInterfaceCAN::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  ROS_INFO("Prepare command mode switch called");
  for (std::string key : stop_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        control_level_[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (std::string key : start_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        ROS_INFO("Command interface switching to %s for joint %s",key.c_str(),info_.joints[i].name.c_str());
        control_level_[i] = integration_level_t::EFFORT;
      }

      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        ROS_INFO("Command interface switching to %s for joint %s",key.c_str(),info_.joints[i].name.c_str());
        control_level_[i] = integration_level_t::VELOCITY;
      }

      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        ROS_INFO("Command interface switching to %s for joint %s",key.c_str(),info_.joints[i].name.c_str());
        control_level_[i] = integration_level_t::POSITION;
      }
    }
  }

  return  hardware_interface::return_type::OK;
}

hardware_interface::return_type ODriveHardwareInterfaceCAN::perform_command_mode_switch(
  const std::vector<std::string> &a, const std::vector<std::string> &b)
{
  ROS_INFO("Perform commnad mode switch called ..a %ld , b%ld", a.size(), b.size());
  for (size_t i = 0; i < info_.joints.size(); i++) {
    float input_torque, input_vel, input_pos;

    switch (control_level_[i]) {
    case integration_level_t::UNDEFINED:
        odrive_can_->can_request_state(can_ids_[i],AXIS_STATE_IDLE);
        break;

      case integration_level_t::EFFORT:
        hw_commands_efforts_[i] = hw_efforts_[i];
        odrive_can_->can_set_control_mode(can_ids_[i],CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
        odrive_can_->can_set_input_torque(can_ids_[i], (hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0)));
        odrive_can_->can_request_state(can_ids_[i],AXIS_STATE_CLOSED_LOOP_CONTROL);
        break;

      case integration_level_t::VELOCITY:
        hw_commands_velocities_[i] = hw_velocities_[i];
        hw_commands_efforts_[i] = 0;
        odrive_can_->can_set_control_mode(can_ids_[i],CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
        input_vel = (hw_commands_velocities_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
        input_torque = hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0);
        odrive_can_->can_set_input_vel_torque(can_ids_[i], input_vel, input_torque);
        odrive_can_->can_request_state(can_ids_[i],AXIS_STATE_CLOSED_LOOP_CONTROL);
        break;

      case integration_level_t::POSITION:
        hw_commands_positions_[i] = hw_positions_[i];
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        odrive_can_->can_set_control_mode(can_ids_[i],CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
        input_pos = (hw_commands_positions_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
        odrive_can_->can_set_position(can_ids_[i],input_pos,0,0);
        odrive_can_->can_request_state(can_ids_[i],AXIS_STATE_CLOSED_LOOP_CONTROL);
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn ODriveHardwareInterfaceCAN::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  ROS_INFO("Start called ...");


  //status_ = hardware_interface::status::STARTED;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterfaceCAN::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  uint8_t requested_state = AXIS_STATE_IDLE;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    odrive_can_->can_request_state(can_ids_[i],AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ODriveHardwareInterfaceCAN::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // read from the can shared variables to the state variables
  for (size_t i = 0; i < info_.sensors.size(); i++) {
    hw_vbus_voltages_[i] = hw_atomics_->can_shared_vbus_voltages_[i].load(std::memory_order_relaxed);
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    hw_positions_[i]  = hw_atomics_->can_shared_positions_[i].load(std::memory_order_relaxed)  * 2 * M_PI * (reverse_control_[i] ? -1.0 : 1.0);
    hw_velocities_[i] = hw_atomics_->can_shared_velocities_[i].load(std::memory_order_relaxed) * 2 * M_PI * (reverse_control_[i] ? -1.0 : 1.0);
    hw_efforts_[i]    =  hw_atomics_->can_shared_efforts_[i].load(std::memory_order_relaxed);
    hw_axis_errors_[i]        = hw_atomics_->can_shared_axis_errors_[i].load(std::memory_order_relaxed);
    hw_motor_temperatures_[i] =  hw_atomics_->can_shared_motor_temperatures_[i].load(std::memory_order_relaxed);
    hw_motor_current_[i]      =  hw_atomics_->can_shared_motor_currents_[i].load(std::memory_order_relaxed);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ODriveHardwareInterfaceCAN::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.joints.size(); i++) {
    float input_torque, input_vel, input_pos;

    switch (control_level_[i]) {
    case integration_level_t::POSITION:
      input_pos = (hw_commands_positions_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
      ROS_DEBUG("::write %ld POSITION %f", i,input_pos);
      odrive_can_->can_set_position(can_ids_[i], input_pos, 0, 0);
      break;
    case integration_level_t::VELOCITY:
      input_vel = (hw_commands_velocities_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
      ROS_DEBUG("::write %ld VELOCITY %f", i,input_vel);
      odrive_can_->can_set_input_vel_torque(can_ids_[i], input_vel, 0.0);
      break;
    case integration_level_t::EFFORT:
      input_torque = hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0);
      ROS_DEBUG("::write %ld EFFORT %f", i,input_torque);
      odrive_can_->can_set_input_torque(can_ids_[i], input_torque);
      break;
    }
  }
  return hardware_interface::return_type::OK;
}
}  // namespace odrive_hardware_interface_can

PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_interface::ODriveHardwareInterfaceCAN, hardware_interface::SystemInterface)

