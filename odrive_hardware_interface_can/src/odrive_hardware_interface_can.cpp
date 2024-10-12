
#include "odrive_hardware_interface_can.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odrive_hardware_interface
{
hardware_interface::CallbackReturn ODriveHardwareInterfaceCAN::on_init(const hardware_interface::HardwareInfo & info)
{
  if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return  hardware_interface::CallbackReturn::ERROR;
  }

  can_ids_.resize(info_.joints.size(),-1);
  axes_.resize(info_.joints.size(),-1);
  vel_gain_.resize(info_.joints.size(),std::numeric_limits<float>::quiet_NaN());
  vel_integrator_gain_.resize(info_.joints.size(),std::numeric_limits<float>::quiet_NaN());
  torque_constants_.resize(info_.joints.size(),-1);
  reverse_control_.resize(info_.joints.size(), false);
  enable_watchdogs_.resize(info_.joints.size());
  
  // state interfaces
  hw_vbus_voltages_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_axis_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_current_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_cmd_vel_zero_.resize(info_.joints.size(),true);
  hw_vel_prev_value_.resize(info_.joints.size(),0.0);
  hw_vel_suppress_.resize(info_.joints.size(),false);
  hw_sent_count_.resize(info_.joints.size(),0);
  hw_received_count_.resize(info_.joints.size(),0);
  // command interfaces
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  /* odrive global parameters */
  for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    odrive_can_.interface_ = std::string(sensor.parameters.at("can_interface"));
    can_speed_ = std::stoi(sensor.parameters.at("can_speed"));
    ROS_INFO(" can_interface %s, can_speed %d",odrive_can_.interface_.c_str(), can_speed_);
  }
  /* axis related parameters */
  /* &&  create the canid <--> axis map */
  for (size_t i = 0 ; i < info_.joints.size(); i++) {
    reverse_control_[i]  = std::stoi(info_.joints[i].parameters.at("reverse_control")) ? true : false;
    axes_[i] = std::stoi(info_.joints[i].parameters.at("axis"));
    can_ids_[i] = std::stoi(info_.joints[i].parameters.at("can_id"));
    vel_gain_[i] = std::stof(info_.joints[i].parameters.at("vel_gain"));
    vel_integrator_gain_[i] = std::stof(info_.joints[i].parameters.at("vel_integrator_gain"));
    canid_axis_[can_ids_[i]] = axes_[i]; 
    ROS_INFO("Can id %d mapped to axis %d reversed %s vel_gain %f vel_integrator gain %f", 
             can_ids_[i], 
             axes_[i], 
             (reverse_control_[i] ? "true" : "false"),
             vel_gain_[i],
             vel_integrator_gain_[i]);
  }
  odrive_can_.can_init(odrive_can_.interface_, can_speed_, &hw_atomics_, &canid_axis_); // open and initialize the can <-> serial interface
  // start the odrive thread
  can_thread_ptr_ = new std::thread(can_thread,&odrive_can_);
  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
  ROS_INFO("Configured");
  return  hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterfaceCAN::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "vbus_voltage", &hw_vbus_voltages_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "axis_error", &hw_axis_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "motor_temperature", &hw_motor_temperatures_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "motor_current", &hw_motor_current_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "sent_count", &hw_sent_count_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "received_count", &hw_received_count_[i]));
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
        ROS_INFO("Command interface switching to EFFORT %s for joint %s",key.c_str(),info_.joints[i].name.c_str());
        control_level_[i] = integration_level_t::EFFORT;
      }

      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        ROS_INFO("Command interface switching to VELOCITY %s for joint %s",key.c_str(),info_.joints[i].name.c_str());
        control_level_[i] = integration_level_t::VELOCITY;
      }

      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        ROS_INFO("Command interface switching to POSITION %s for joint %s",key.c_str(),info_.joints[i].name.c_str());
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
        odrive_can_.can_request_state(can_ids_[i],AXIS_STATE_IDLE);
        break;

      case integration_level_t::EFFORT:
        hw_commands_efforts_[i] = 0; // hw_efforts_[i];
        odrive_can_.can_set_control_mode(can_ids_[i],CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
        //odrive_can_.can_set_input_torque(can_ids_[i], (hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0)));
        odrive_can_.can_request_state(can_ids_[i],AXIS_STATE_CLOSED_LOOP_CONTROL);
        break;

      case integration_level_t::VELOCITY:
        hw_commands_velocities_[i] = 0; // hw_velocities_[i];
        hw_commands_efforts_[i] = 0;
        odrive_can_.can_set_control_mode(can_ids_[i],CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
        input_vel = (hw_commands_velocities_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
        input_torque = hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0);
        //odrive_can_.can_set_input_vel_torque(can_ids_[i], input_vel, input_torque);
        odrive_can_.can_request_state(can_ids_[i],AXIS_STATE_CLOSED_LOOP_CONTROL);
        break;

      case integration_level_t::POSITION:
        hw_commands_positions_[i] = hw_positions_[i];
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        odrive_can_.can_set_control_mode(can_ids_[i],CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
        input_pos = (hw_commands_positions_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
        //odrive_can_.can_set_position(can_ids_[i],input_pos,0,0);
        odrive_can_.can_request_state(can_ids_[i],AXIS_STATE_CLOSED_LOOP_CONTROL);
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn ODriveHardwareInterfaceCAN::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  ROS_INFO("Activate called ...");
  // reset positions to zero
  for (size_t i = 0; i < info_.joints.size(); i++) {
    odrive_can_.can_set_absolute_position(can_ids_[i],0.0);
    odrive_can_.can_set_vel_gains(can_ids_[i],vel_gain_[i], vel_integrator_gain_[i]);
  }
  ROS_INFO("Absolute position reset ...");
  //status_ = hardware_interface::status::STARTED;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterfaceCAN::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  ROS_INFO("Deactivate callled");
  for (size_t i = 0; i < info_.joints.size(); i++) {
    odrive_can_.can_request_state(can_ids_[i],AXIS_STATE_IDLE);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ODriveHardwareInterfaceCAN::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // read from the can shared variables to the state variables
  for (size_t i = 0; i < info_.sensors.size(); i++) {
    hw_vbus_voltages_[i] = hw_atomics_.vbus_voltages_[i].load(std::memory_order_relaxed);
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    hw_positions_[i]  = hw_atomics_.positions_[i].load(std::memory_order_relaxed)  * 2 * M_PI * (reverse_control_[i] ? -1.0 : 1.0);
    hw_velocities_[i] = hw_atomics_.velocities_[i].load(std::memory_order_relaxed) * 2 * M_PI * (reverse_control_[i] ? -1.0 : 1.0);
    hw_efforts_[i]    =  hw_atomics_.efforts_[i].load(std::memory_order_relaxed);
    hw_axis_errors_[i]        = hw_atomics_.axis_errors_[i].load(std::memory_order_relaxed);
    hw_motor_temperatures_[i] =  hw_atomics_.motor_temperatures_[i].load(std::memory_order_relaxed);
    hw_motor_current_[i]      =  hw_atomics_.motor_currents_[i].load(std::memory_order_relaxed);
    if (hw_cmd_vel_zero_[i]) {
      if (!hw_vel_suppress_[i]) {
        // then check for jitter (sign changed): or it came very close to zero
        if (std::signbit(hw_velocities_[i]) != std::signbit(hw_vel_prev_value_[i]) ||
            abs(hw_velocities_[i]) < 0.0000001) {
          hw_vel_suppress_[i] = true;
          hw_velocities_[i] = 0.0;
        }
      } else {
        hw_velocities_[i] = 0.0;
      }
    } else {
      hw_vel_suppress_[i] = false;
    }
    hw_vel_prev_value_[i] = hw_velocities_[i];
    hw_sent_count_[i] = hw_atomics_.sent_count_[i].load(std::memory_order_relaxed);
    hw_received_count_[i] = hw_atomics_.received_count_[i].load(std::memory_order_relaxed);
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
      odrive_can_.can_set_position(can_ids_[i], input_pos, 0, 0);
      break;
    case integration_level_t::VELOCITY:
      input_vel = (hw_commands_velocities_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
      ROS_DEBUG("::write %ld VELOCITY %f", i,input_vel);
      odrive_can_.can_set_input_vel_torque(can_ids_[i], input_vel, 0.0);
      // record if we are asking it to stop
      if (abs(hw_commands_velocities_[i]) < 0.0000001) {
        hw_cmd_vel_zero_[i] = true;
      } else {
        hw_cmd_vel_zero_[i] = false;
      }
      break;
    case integration_level_t::EFFORT:
      input_torque = hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0);
      ROS_DEBUG("::write %ld EFFORT %f", i,input_torque);
      odrive_can_.can_set_input_torque(can_ids_[i], input_torque);
      break;
    }
  }
  return hardware_interface::return_type::OK;
}
}  // namespace odrive_hardware_interface_can

PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_interface::ODriveHardwareInterfaceCAN, hardware_interface::SystemInterface)

