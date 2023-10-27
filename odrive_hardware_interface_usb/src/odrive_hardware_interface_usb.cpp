#include <odrive.hpp>

#include "odrive_hardware_interface_usb.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace odrive_hardware_interface
{
hardware_interface::CallbackReturn ODriveHardwareInterfaceUSB::on_init(const hardware_interface::HardwareInfo & info)
{
  if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return  hardware_interface::CallbackReturn::ERROR;
  }

  serial_numbers_.resize(2);
  torque_constants_.resize(info_.joints.size());
  reverse_control_.resize(info_.joints.size(), false);
  enable_watchdogs_.resize(info_.joints.size());
  hw_vbus_voltages_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_axis_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_encoder_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_controller_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_fet_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_current_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_vel_set_zero_.resize(info_.joints.size(), true);
  hw_vel_prev_value_.resize(info_.joints.size(),0.0);
  hw_vel_suppress_.resize(info_.joints.size(),false);
  /* odrive global parameters */
  for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    serial_numbers_[0].emplace_back(std::stoull(sensor.parameters.at("serial_number"), 0, 16));
  }
  /* axis related parameters */
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    serial_numbers_[1].emplace_back(std::stoull(joint.parameters.at("serial_number"), 0, 16));
    axes_.emplace_back(std::stoi(joint.parameters.at("axis")));
  }

  /* instantiate and initialize the odrives */
  for (const uint64_t serial_num : serial_numbers_[0]) {
    odrives[serial_num] = new odrive();
    odrives[serial_num]->endpoint = new odrive_endpoint();
    odrives[serial_num]->endpoint->init(serial_num);
    auto od = odrives[serial_num];
    // Read JSON from target
    Json::Value odrive_json;
    if (getJson(od->endpoint, &odrive_json)) {
      return  hardware_interface::CallbackReturn::SUCCESS;
    } 
    odrives[serial_num]->json = odrive_json;   
  }
  for (size_t i = 0; i < info_.joints.size(); i++) {
    float torque_constant;
    uint64_t serial_num = serial_numbers_[1][i];
    odrive *od = odrives[serial_num];
    std::string axis = string("axis")+std::to_string(axes_[i]);
    readOdriveData(od->endpoint, od->json,axis + ".motor.config.torque_constant",torque_constant);
    torque_constants_[i] = torque_constant;
    enable_watchdogs_[i] = std::stoi(info_.joints[i].parameters.at("enable_watchdog")) ? true : false;
    reverse_control_[i]  = std::stoi(info_.joints[i].parameters.at("reverse_control")) ? true : false;
    if (enable_watchdogs_[i]) {
      float wd_timeout = std::stof(info_.joints[i].parameters.at("watchdog_timeout"));
      writeOdriveData(od->endpoint, od->json, axis + ".config.watchdog_timeout", wd_timeout);
    }
    bool enb_wd = (bool)enable_watchdogs_[i];
    writeOdriveData(od->endpoint, od->json, axis + ".config.enable_watchdog", enb_wd);
  }

  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
  //status_ = hardware_interface::status::CONFIGURED;
  ROS_INFO("Configured");
  return  hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterfaceUSB::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.sensors.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[i].name, "vbus_voltage", &hw_vbus_voltages_[i]));
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "axis_error", &hw_axis_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "motor_error", &hw_motor_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "encoder_error", &hw_encoder_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "controller_error", &hw_controller_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "fet_temperature", &hw_fet_temperatures_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "motor_temperature", &hw_motor_temperatures_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "motor_current", &hw_motor_temperatures_[i]));
  }
  ROS_INFO("States exported ..");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ODriveHardwareInterfaceUSB::export_command_interfaces()
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

 hardware_interface::return_type ODriveHardwareInterfaceUSB::prepare_command_mode_switch(
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

hardware_interface::return_type ODriveHardwareInterfaceUSB::perform_command_mode_switch(
  const std::vector<std::string> &a, const std::vector<std::string> &b)
{
  ROS_INFO("Perform commnad mode switch called ..a %ld , b%ld", a.size(), b.size());
  for (size_t i = 0; i < info_.joints.size(); i++) {
    float input_torque, input_vel, input_pos;
    uint8_t requested_state;
    std::string axis = "axis" + std::to_string(axes_[i]);
    uint64_t serial_num = serial_numbers_[1][i];
    odrive *od = odrives[serial_num];
    uint8_t control_mode = (uint8_t)control_level_[i];
    
    switch (control_level_[i]) {
      case integration_level_t::UNDEFINED:
        requested_state = AXIS_STATE_IDLE;
        writeOdriveData(od->endpoint, od->json, axis + ".requested_state",requested_state);
        break;

      case integration_level_t::EFFORT:
        hw_commands_efforts_[i] = hw_efforts_[i];
        writeOdriveData(od->endpoint, od->json, axis + ".controller.config.control_mode",control_mode);
        input_torque = (hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0));
        writeOdriveData(od->endpoint,od->json, axis + ".controller.input_torque",input_torque);
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        writeOdriveData(od->endpoint, od->json, axis + ".requested_state",requested_state);
        break;

      case integration_level_t::VELOCITY:
        hw_commands_velocities_[i] = hw_velocities_[i];
        hw_commands_efforts_[i] = 0;
        writeOdriveData(od->endpoint, od->json, axis + ".controller.config.control_mode",control_mode);
        input_vel = (hw_commands_velocities_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
        writeOdriveData(od->endpoint, od->json, axis + ".controller.input_vel",input_vel);
        input_torque = hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0);
        writeOdriveData(od->endpoint, od->json, axis + ".controller.input_torque",input_torque);
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        writeOdriveData(od->endpoint, od->json, axis + ".requested_state",requested_state);
        break;

      case integration_level_t::POSITION:
        hw_commands_positions_[i] = hw_positions_[i];
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        writeOdriveData(od->endpoint, od->json, axis + ".controller.config.control_mode",control_mode);
        input_pos = (hw_commands_positions_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
        writeOdriveData(od->endpoint, od->json, axis + ".controller.input_pos", input_pos);
        input_vel = (hw_commands_velocities_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
        writeOdriveData(od->endpoint, od->json, axis + ".controller.input_vel", input_vel);
        input_torque = hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0);
        writeOdriveData(od->endpoint, od->json, axis + ".controller.input_torque", input_torque);
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        writeOdriveData(od->endpoint, od->json, axis + ".requested_state",requested_state);
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn ODriveHardwareInterfaceUSB::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  ROS_INFO("Start called ...");
  // for all drives 
  for (auto serial_num : serial_numbers_[0]) {
    odrive *od = odrives[serial_num];
    execOdriveFunc(od->endpoint, od->json, "clear_errors");
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    std::string axis = "axis" + std::to_string(axes_[i]);
    uint64_t serial_num = serial_numbers_[1][i];
    odrive *od = odrives[serial_num];    
    ROS_INFO("Starting AXIS %ld watchdog_enabled (%s) reverse_control(%s)",i, 
             enable_watchdogs_[i] ? "true" : "false",
             reverse_control_[i] ? "true" : "false");
    if (enable_watchdogs_[i]) {
      execOdriveFunc(od->endpoint, od->json, axis + ".watchdog_feed");
    }
    //uint8_t requested_state = AXIS_STATE_MOTOR_CALIBRATION;
    //writeOdriveData(od->endpoint, od->json, axis + ".requested_state", requested_state);
  }

  //status_ = hardware_interface::status::STARTED;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterfaceUSB::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  uint8_t requested_state = AXIS_STATE_IDLE;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    std::string axis = "axis" + std::to_string(axes_[i]);
    uint64_t serial_num = serial_numbers_[1][i];
    odrive *od = odrives[serial_num];    
    writeOdriveData(od->endpoint, od->json, axis + ".requested_state",requested_state);
  }

  //status_ = hardware_interface::status::STOPPED;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ODriveHardwareInterfaceUSB::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.sensors.size(); i++) {
    float vbus_voltage;
    uint64_t serial_num = serial_numbers_[0][i];
    odrive *od = odrives[serial_num];    
    //readOdriveData(od->endpoint, od->json, "vbus_voltage", vbus_voltage);
    //hw_vbus_voltages_[i] = vbus_voltage;
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    std::string axis = "axis" + std::to_string(axes_[i]);
    uint64_t serial_num = serial_numbers_[1][i];
    odrive *od = odrives[serial_num];    
    float Iq_measured, vel_estimate, pos_estimate, fet_temperature, motor_temperature, motor_current;
    int32_t axis_error;
    uint8_t controller_error;
    uint64_t  motor_error ;
    uint16_t encoder_error;
    //readOdriveData(od->endpoint, od->json, axis + ".motor.current_control.Iq_measured",Iq_measured);
    //hw_efforts_[i] = Iq_measured * torque_constants_[i];
    if (control_level_[i] == integration_level_t::POSITION) {
      readOdriveData(od->endpoint, od->json, axis + ".encoder.pos_estimate", pos_estimate);
      hw_positions_[i] = pos_estimate * 2 * M_PI * (reverse_control_[i] ? -1.0 : 1.0);
    } else { // if (control_level_[i] == integration_level_t::VELOCITY) {
      readOdriveData(od->endpoint, od->json, axis + ".encoder.vel_estimate", vel_estimate);
      // if requested to go to zero
      hw_velocities_[i] = vel_estimate * 2 * M_PI * (reverse_control_[i] ? -1.0 : 1.0);
      if (hw_vel_set_zero_[i]) {
          if (!hw_vel_suppress_[i]) {
            // then check for jitter (sign changed)
            if (std::signbit(hw_velocities_[i]) != std::signbit(hw_vel_prev_value_[i])) {
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

      //ROS_INFO("Reading velocity %f %f",hw_velocities_[i],vel_estimate);
    }
    //readOdriveData(od->endpoint, od->json, axis + ".error", axis_error);
    //hw_axis_errors_[i] = axis_error;
    //readOdriveData(od->endpoint, od->json, axis + ".motor.error", motor_error);
    //hw_motor_errors_[i] = motor_error;
    //readOdriveData(od->endpoint, od->json, axis + ".encoder.error", encoder_error);
    //hw_encoder_errors_[i] = encoder_error;
    //readOdriveData(od->endpoint, od->json, axis + ".controller.error", controller_error);
    //hw_controller_errors_[i] = controller_error;
    //readOdriveData(od->endpoint, od->json, axis + ".motor.fet_thermistor.temperature", fet_temperature);
    //hw_fet_temperatures_[i] = fet_temperature;
    //readOdriveData(od->endpoint, od->json, axis + ".motor.motor_thermistor.temperature",motor_temperature);
    //hw_motor_temperatures_[i] = motor_temperature;
    //readOdriveData(od->endpoint, od->json, axis + ".motor.I_bus",motor_current);
    //hw_motor_current_[i] = motor_current;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ODriveHardwareInterfaceUSB::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.joints.size(); i++) {
    std::string axis = "axis" + std::to_string(axes_[i]);
    uint64_t serial_num = serial_numbers_[1][i];
    odrive *od = odrives[serial_num];    
    float input_torque, input_vel, input_pos;

    switch (control_level_[i]) {
      case integration_level_t::POSITION:
        input_pos = (hw_commands_positions_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
        ROS_DEBUG("::write %ld POSITION %f", i,input_pos);
        writeOdriveData(od->endpoint, od->json, axis + ".controller.input_pos",input_pos);
        break;
      case integration_level_t::VELOCITY:
        input_vel = (hw_commands_velocities_[i] / 2 / M_PI) * (reverse_control_[i] ? -1.0 : 1.0);
        ROS_DEBUG("::write %ld VELOCITY %f", i,input_vel);
        // record if we are asking it to stop
        if (abs(hw_commands_velocities_[i]) < 0.0000001) {
          hw_vel_set_zero_[i] = true;
        } else {
          hw_vel_set_zero_[i] = false;
        }
        writeOdriveData(od->endpoint, od->json, axis + ".controller.input_vel",input_vel);
        break;
      case integration_level_t::EFFORT:
        input_torque = hw_commands_efforts_[i] * (reverse_control_[i] ? -1.0 : 1.0);
        ROS_DEBUG("::write %ld EFFORT %f", i,input_torque);
        writeOdriveData(od->endpoint, od->json, axis + ".controller.input_torque",input_torque);
        break;
      case integration_level_t::UNDEFINED:
        if (enable_watchdogs_[i]) {
          execOdriveFunc(od->endpoint, od->json, axis + ".watchdog_feed");
        }
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_interface::ODriveHardwareInterfaceUSB, hardware_interface::SystemInterface)

