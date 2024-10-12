#include "odrive_can.hpp"

using namespace std;

/**
/// @brief can_adapter init . will open the tty port and set the baud rate
/// @param tty - name of the tty eg /dev/ttyUSB0 
/// @param baud - 250000
/// @return the file descriptor for the tty
*/
int odrive_can::can_adapter_init(std::string & can_interface) {
    int socket_id_ = socket(PF_CAN, SOCK_RAW , CAN_RAW);
    if (socket_id_ == -1) {
        ROS_WARN("Failed to create socket");
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_.c_str());
    if (ioctl(socket_id_, SIOCGIFINDEX, &ifr) == -1) {
        ROS_WARN("Failed to get interface index");
        close(socket_id_);
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_id_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
        ROS_WARN("Failed to bind socket");
        close(socket_id_);
        return false;
    }

    struct msghdr message = {
        .msg_name = nullptr,
        .msg_namelen = 0,
        .msg_iov = nullptr,
        .msg_iovlen = 0,
        .msg_control = nullptr,
        .msg_controllen = 0,
        .msg_flags = 0
    };

    int retcode = recvmsg(socket_id_, &message, 0);
    if (retcode < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        ROS_WARN("Failed to recvmsg");
        close(socket_id_);
        socket_id_ = 0;
        return false;
    }

    return socket_id_;
}

// The main function assumed to be running as a thread
void can_thread(odrive_can *oc) {
    struct pollfd fds[1];
    int poll_r;
    // wait for activaton
    while (!oc->hw_atomics_->active.load(std::memory_order_relaxed)) usleep(10);  
    while (oc->hw_atomics_->active.load(std::memory_order_relaxed)) {
        int32_t can_id, cmd_id;
        can_frame frame ;
        struct cmsghdr ctrlmsg;
        struct iovec vec = {.iov_base = &frame, .iov_len = sizeof(frame)};
        struct msghdr message = {
            .msg_name = nullptr,
            .msg_namelen = 0,
            .msg_iov = &vec, 
            .msg_iovlen = 1,
            .msg_control = &ctrlmsg,
            .msg_controllen = sizeof(ctrlmsg),
            .msg_flags = 0
            };
        ssize_t n_received = read(oc->can_fd_, &frame, sizeof(can_frame));
        if (n_received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                ROS_WARN("no message received fd %d %s", oc->can_fd_, strerror(errno));
                continue;
            } else {
                ROS_WARN("Socket read failed");
                continue;
            }
        }
        if (n_received < static_cast<ssize_t>(sizeof(struct can_frame))) {
            ROS_WARN("invalid message length %d ",(int)n_received);
            continue;
        }
        can_id = frame.can_id >> 5;
        cmd_id = frame.can_id & 0x1f;

        ROS_DEBUG("received packet %02x %02x",can_id,cmd_id);
        // if we cannot determine the frame then continue
        if (cmd_id == -1) continue;
        int axis = (*(oc->canid_axis_))[can_id];
        oc->hw_atomics_->received_count_[axis]++;
        switch(cmd_id) {
        case CmdId::kHeartbeat: {
            oc->hw_atomics_->active_errors_       [axis] = read_le<uint32_t>(frame.data + 0);
            oc->hw_atomics_->axis_state_          [axis] = read_le<uint8_t>(frame.data + 4);
            oc->hw_atomics_->procedure_result_    [axis] = read_le<uint8_t>(frame.data + 5);
            oc->hw_atomics_->trajectory_done_flag_[axis] = read_le<bool>(frame.data + 6);
            if (oc->hw_atomics_->axis_debug_[axis]) {
                ROS_INFO("received heartbeat axis %d state %d active_errors 0x%x",
                         axis,
                         read_le<uint32_t>(frame.data + 0),
                         read_le<uint32_t>(frame.data + 4));
            }
            break;
        }
        case CmdId::kGetError: {
            oc->hw_atomics_->active_errors_ [axis] = read_le<uint32_t>(frame.data + 0);
            oc->hw_atomics_->disarm_reason_ [axis] = read_le<uint32_t>(frame.data + 4);
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            oc->hw_atomics_->positions_  [axis] = read_le<float>(frame.data + 0);
            oc->hw_atomics_->velocities_ [axis] = read_le<float>(frame.data + 4);
            // clamp velocities within reasonable values
            if (oc->hw_atomics_->velocities_[axis] > 10.0 || oc->hw_atomics_->velocities_[axis] < -10.0)
                oc->hw_atomics_->velocities_[axis] = 0.0;
            ROS_DEBUG("kGetEncoderEstimates asix %d pos %f vel %f",
                     axis, 
                     oc->hw_atomics_->positions_[axis].load(std::memory_order_relaxed),
                     oc->hw_atomics_->velocities_[axis].load(std::memory_order_relaxed));
            break;
        }
        case CmdId::kGetIq: {
            //iq_setpoint = read_le<float>(frame.data + 0);
            oc->hw_atomics_->motor_currents_ [axis] = read_le<float>(frame.data + 4);
            break;
        }
        case CmdId::kGetTemp: {
            oc->hw_atomics_->fet_temperatures_   [axis]= read_le<float>(frame.data + 0);
            oc->hw_atomics_->motor_temperatures_ [axis]= read_le<float>(frame.data + 4);
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            oc->hw_atomics_->vbus_voltages_ [axis]= read_le<float>(frame.data + 0);
            oc->hw_atomics_->vbus_currents_ [axis]= read_le<float>(frame.data + 4);
            break;
        }
        case CmdId::kGetTorques: {
            oc->hw_atomics_->torque_targets_ [axis] = read_le<float>(frame.data + 0);
            oc->hw_atomics_->efforts_        [axis] = read_le<float>(frame.data + 4);
            break;
        }
        default: {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "Received unused message: ID = 0x%x", (cmd_id & 0x1F));
            break;
        }
        }
    }
}

/**
 * @brief encode and send a can frame to the can <--> serial interface
 * 
 * @param can_id    - can_id for the endpoint device 
 * @param cmd_id    - command id to send
 * @param frame     - pointer to a can frame
 * @param data_len  - data_len (number of bytes in .data field in can_frame)
 * @return int      - number of byte written -1 in case of error
 */
int odrive_can::can_send_frame(uint32_t can_id, uint32_t cmd_id, can_frame *frame, int32_t data_len) {
    // encode
    frame->can_id = can_id << 5 | cmd_id;
    frame->can_dlc = data_len;
    int bytes = write(can_fd_,frame, sizeof(can_frame));
    if (bytes != sizeof(can_frame)) {
        ROS_ERROR("can_send_frame error %d %s",can_fd_,strerror(errno));
    }
    int axis = (*(canid_axis_))[can_id];
    hw_atomics_->sent_count_[axis]++;
    return bytes;
}

// set commands

/**
 * @brief send requested state to a odrive Axis (see enum AxisState)
 * 
 * @param can_id - can_id of the axis 
 * @param state  - requested state
 */
void odrive_can::can_request_state(int32_t can_id, uint32_t state) {
    can_frame cf_ ;
    memset(&cf_,0,sizeof(cf_));
    write_le<uint32_t>(state, cf_.data);
    can_send_frame(can_id,CmdId::kSetAxisState,&cf_,4);
    // wait for the axis to switch to state
    int axis = (*(canid_axis_))[can_id];
    ROS_INFO("waiting for axis %d to goto state %d", axis,state);
    hw_atomics_->axis_debug_[axis] = true;
    while (hw_atomics_->axis_state_[axis].load(std::memory_order_relaxed) != state) {
        usleep(10);   
    }
    hw_atomics_->axis_debug_[axis] = false;
}

/**
 * @brief send control mode frame
 * 
 * @param can_id        - can_id of the axis 
 * @param control_mode  - control mode (see enum ControlMode)
 * @param input_mode    - input mode (see enum InputMode)
 */
void odrive_can::can_set_control_mode(int32_t can_id, uint32_t control_mode , uint32_t input_mode ) {
    can_frame cf_;
    memset(&cf_,0,sizeof(cf_));
    write_le<uint32_t>(control_mode, cf_.data);
    write_le<uint32_t>(input_mode,   cf_.data + 4);
    can_send_frame(can_id,CmdId::kSetControllerMode,&cf_,8);
}

/**
 * @brief set the torque for an axis
 * 
 * @param can_id        - can_id of the axis
 * @param input_torque  - inpute torque value
 */
void odrive_can::can_set_input_torque(int32_t can_id, float input_torque) {
    can_frame cf_;
    memset(&cf_,0,sizeof(cf_));
    write_le<float>(input_torque, cf_.data);
    can_send_frame(can_id,CmdId::kSetInputTorque,&cf_,4);
}

/**
 * @brief set the input velocity for a odrive axis
 * 
 * @param can_id        - can_id of the axis
 * @param input_vel     - target input velocity 
 * @param input_torque  - input torque 
 */
void odrive_can::can_set_input_vel_torque(int32_t can_id, float input_vel, float input_torque) {
    can_frame cf_;
    memset(&cf_,0,sizeof(cf_));
    write_le<float>(input_vel,    cf_.data);
    write_le<float>(input_torque, cf_.data + 4);
    can_send_frame(can_id, CmdId::kSetInputVel, &cf_,8);
}

/**
 * @brief set the poistion for an axis
 * 
 * @param can_id        - can_id of the axis
 * @param input_pos     - target poistion
 * @param input_vel     - velocity
 * @param input_torque  - torque
 */
void odrive_can::can_set_position(int32_t can_id, float input_pos, uint8_t input_vel, uint8_t input_torque) {
    can_frame cf_;
    memset(&cf_,0,sizeof(cf_));
    write_le<float>(input_pos,  cf_.data);
    write_le<int8_t>(((int8_t)((input_vel) * 1000)), cf_.data + 4);
    write_le<int8_t>(((int8_t)((input_torque) * 1000)), cf_.data + 6);
    can_send_frame(can_id, CmdId::kSetInputPos, &cf_,8);
}

void odrive_can::can_set_absolute_position(int32_t can_id, float abs_pos) {
    can_frame cf_;
    memset(&cf_,0,sizeof(cf_));
    write_le<float>(abs_pos,  cf_.data);
    can_send_frame(can_id, CmdId::kSetAbsolutePos, &cf_,4);
}

void odrive_can::can_set_vel_gains(int32_t can_id, float vel_gain, float vel_integrator_gain) {
    can_frame cf_;
    memset(&cf_,0,sizeof(cf_));
    write_le<float>(vel_gain,cf_.data);
    write_le<float>(vel_integrator_gain,cf_.data+4);
    can_send_frame(can_id, CmdId::kSetVelGains, &cf_, 8);  
}