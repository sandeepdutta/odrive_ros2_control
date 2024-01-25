#include "odrive_can.hpp"

using namespace std;


static CANUSB_SPEED canusb_int_to_speed(int speed)
{
    switch (speed) {
    case 1000000:
        return CANUSB_SPEED_1000000;
    case 800000:
        return CANUSB_SPEED_800000;
    case 500000:
        return CANUSB_SPEED_500000;
    case 400000:
        return CANUSB_SPEED_400000;
    case 250000:
        return CANUSB_SPEED_250000;
    case 200000:
        return CANUSB_SPEED_200000;
    case 125000:
        return CANUSB_SPEED_125000;
    case 100000:
        return CANUSB_SPEED_100000;
    case 50000:
        return CANUSB_SPEED_50000;
    case 20000:
        return CANUSB_SPEED_20000;
    case 10000:
        return CANUSB_SPEED_10000;
    case 5000:
        return CANUSB_SPEED_5000;
    default:
        return CANUSB_SPEED_250000;
    }
}

static int generate_checksum(const unsigned char *data, int data_len)
{
    int checksum = 0;
    for (int i = 0; i < data_len; i++) {
        checksum += data[i];
    }
    return checksum & 0xff;
}

static int command_settings(int tty_fd, CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frame)
{
    int cmd_frame_len;
    unsigned char cmd_frame[20];
    
    cmd_frame_len = 0;
    cmd_frame[cmd_frame_len++] = 0xaa;
    cmd_frame[cmd_frame_len++] = 0x55;
    cmd_frame[cmd_frame_len++] = 0x12;
    cmd_frame[cmd_frame_len++] = speed;
    cmd_frame[cmd_frame_len++] = frame;
    cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
    cmd_frame[cmd_frame_len++] = mode;
    cmd_frame[cmd_frame_len++] = 0x01;
    cmd_frame[cmd_frame_len++] = 0;
    cmd_frame[cmd_frame_len++] = 0;
    cmd_frame[cmd_frame_len++] = 0;
    cmd_frame[cmd_frame_len++] = 0;
    cmd_frame[cmd_frame_len++] = generate_checksum(&cmd_frame[2], 17);
    
    if (write(tty_fd, cmd_frame, cmd_frame_len) < 0) {
        ROS_ERROR("error setting can parameters %s",strerror(errno));
        return -1;
    }
    
    return 0;
}

/**
/// @brief can_adapter init . will open the tty port and set the baud rate
/// @param tty - name of the tty eg /dev/ttyUSB0 
/// @param baud - 250000
/// @return the file descriptor for the tty
*/
int odrive_can::can_adapter_init(const char *tty, int baud, int can_speed) {
    int tty_fd, result;
    struct termios2 tio;

    tty_fd = open(tty, O_RDWR | O_NOCTTY | O_NONBLOCK | O_SYNC);
    if (tty_fd < 0) {
        ROS_ERROR("open(%s) failed: %s", tty, strerror(errno));
        return -1;
    }

    result = ioctl(tty_fd, TCGETS2, &tio);
    if (result < 0) {
        ROS_ERROR("ioctl() failed: %s", strerror(errno));
        close(tty_fd);
        return -1;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag = BOTHER | CS8 | CSTOPB;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_ispeed = baud;
    tio.c_ospeed = baud;
    
    result = ioctl(tty_fd, TCSETS2, &tio);
    if (result < -1) {
        ROS_ERROR("ioctl() failed: %s", strerror(errno));
        close(tty_fd);
        return -1;
    }
    
    command_settings(tty_fd, canusb_int_to_speed(can_speed), CANUSB_MODE_NORMAL, CANUSB_FRAME_STANDARD);
    return tty_fd;
}

/**
 * @brief will return true when a complete frame has been received
 *        
 * @param frame         frame data received so far 
 * @param frame_len     frame len received so far
 * @return true         complete frame received
 * @return false        frame not complete yet
 */
bool odrive_can::can_frame_complete(const uint8_t *frame, int frame_len)
{
    if (frame_len > 0) {
        if (frame[0] != 0xaa) {
            /* Need to sync on 0xaa at start of frames, so just skip. */
            return true;
        }
    }
    // need DLC to determine frame type.
    if (frame_len < 2) {
        return false;
    }

    if (frame[1] == 0x55) { /* Command frame... */
        if (frame_len >= 20) { /* ...always 20 bytes. */
            return true;
        } else {
            return false;
        }
    } else if ((frame[1] >> 4) == 0xc) { /* Data frame... */
        if (frame_len >= (frame[1] & 0xf) + 5) { /* ...payload and 5 bytes. */
            return true;
        } else {
            return false;
        }
    }

    /* Unhandled frame type. */
    return true;
}

/**
 * @brief receive a complete frm from the can interface
 * 
 * @param can_id    reference to return can_id for data frames -1 if command frame
 * @param cmd_id    reference to return cmd_id for data frames
 * @param frame     frame data received
 * @return int      number of bytes in the frame
 */
int odrive_can::can_recv_frame(int32_t &can_id, int32_t &cmd_id, uint8_t *frame)
{
    int result, frame_len, checksum;
    unsigned char byte;

    frame_len = 0;
    while (1) {
        result = read(can_fd_, &byte, 1);
        if (result == -1) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ROS_ERROR ("can_recv_frame() failed: %s\n", strerror(errno));
                return -1;
            }
        } else if (result > 0) {
            if (frame_len == MAX_CAN_FRAME_SIZE) {
                ROS_ERROR("can_recv_frame() failed: Overflow\n");
                return -1;
            }
            frame[frame_len++] = byte;
            if (can_frame_complete(frame, frame_len)) {
                break;
            }
        }
    }
    can_id = -1;
    cmd_id = -1;
    /* Compare checksum for command frames only. */
    if ((frame_len == 20) && (frame[0] == 0xaa) && (frame[1] == 0x55)) {
        checksum = generate_checksum(&frame[2], 17);
        if (checksum != frame[frame_len - 1]) {
            ROS_ERROR ("can_recv_frame() failed: Checksum incorrect\n");
            return -1;
        }
    } else if ((frame_len >= 6) && (frame[0] == 0xaa) && ((frame[1] >> 4) == 0xc)) {  // Data frame
        uint16_t frame_id = read_le<uint16_t>(&frame[2]);
        can_id = frame_id >> 5;
        cmd_id = frame_id & 0x1f;
    }
    return frame_len;
}

// The main function assumed to be running as a thread
void can_thread(odrive_can *oc) {
    struct pollfd fds[1];
    int poll_r;
    // wait for activaton
    while (!oc->hw_atomics_->active.load(std::memory_order_relaxed)) 
        usleep(10);  
    while (oc->hw_atomics_->active.load(std::memory_order_relaxed)) {
        int32_t can_id, cmd_id;
        can_frame can_frame_ ;
        fds[0].fd = oc->can_fd_;
        fds[0].events = POLLIN;
        // wait for 1 second for an event from the can interface
        if ((poll_r =  poll( fds, 1, 1000)) < 0) {
            ROS_ERROR("poll error  %s",strerror(errno));
            continue;
        }
        if (poll_r == 0) {
            ROS_WARN("poll timedout retrying %0x", oc->hw_atomics_->active_errors_[0].load(std::memory_order_relaxed));
            continue;
        }
        // file description ready to read
        int frame_len = oc->can_recv_frame(can_id, cmd_id, (uint8_t *)&can_frame_);
        if (frame_len < 0) {
            ROS_ERROR("frame receive error");
            continue;
        }
        ROS_DEBUG("received packet %02x %02x",can_id,cmd_id);
        // if we cannot determine the frame then continue
        if (cmd_id == -1) continue;
        int axis = (*(oc->canid_axis_))[can_id];
        if (!verify_length("response", 8, frame_len-5)) continue;
        switch(cmd_id) {
        case CmdId::kHeartbeat: {
            oc->hw_atomics_->active_errors_       [axis] = read_le<uint32_t>(can_frame_._data + 0);
            oc->hw_atomics_->axis_state_          [axis] = read_le<uint8_t>(can_frame_._data + 4);
            oc->hw_atomics_->procedure_result_    [axis] = read_le<uint8_t>(can_frame_._data + 5);
            oc->hw_atomics_->trajectory_done_flag_[axis] = read_le<bool>(can_frame_._data + 6);
            if (oc->hw_atomics_->axis_debug_[axis]) {
                ROS_DEBUG("received heartbeat axis %d state %d active_errors 0x%x",
                         axis,
                         read_le<uint32_t>(can_frame_._data + 0),
                         read_le<uint32_t>(can_frame_._data + 4));
            }
            break;
        }
        case CmdId::kGetError: {
            oc->hw_atomics_->active_errors_ [axis] = read_le<uint32_t>(can_frame_._data + 0);
            oc->hw_atomics_->disarm_reason_ [axis] = read_le<uint32_t>(can_frame_._data + 4);
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            oc->hw_atomics_->positions_  [axis] = read_le<float>(can_frame_._data + 0);
            oc->hw_atomics_->velocities_ [axis] = read_le<float>(can_frame_._data + 4);
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
            oc->hw_atomics_->motor_currents_ [axis] = read_le<float>(can_frame_._data + 4);
            break;
        }
        case CmdId::kGetTemp: {
            oc->hw_atomics_->fet_temperatures_   [axis]= read_le<float>(can_frame_._data + 0);
            oc->hw_atomics_->motor_temperatures_ [axis]= read_le<float>(can_frame_._data + 4);
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            oc->hw_atomics_->vbus_voltages_ [axis]= read_le<float>(can_frame_._data + 0);
            oc->hw_atomics_->vbus_currents_ [axis]= read_le<float>(can_frame_._data + 4);
            break;
        }
        case CmdId::kGetTorques: {
            oc->hw_atomics_->torque_targets_ [axis] = read_le<float>(can_frame_._data + 0);
            oc->hw_atomics_->efforts_        [axis] = read_le<float>(can_frame_._data + 4);
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
 * @param data_len  - data_len (number of bytes in ._data field in can_frame)
 * @return int      - number of byte written -1 in case of error
 */
int odrive_can::can_send_frame(uint32_t can_id, uint32_t cmd_id, can_frame *frame, int32_t data_len) {
    // encode
    frame->_sof = 0xaa;
    frame->_dlc = 0xc0|data_len;
    write_le<uint16_t>((can_id << 5) | cmd_id, (uint8_t*)(&frame->_frame_id));
    frame->_data[data_len] = 0x55;
    int bytes = write(can_fd_,frame, data_len+5);
    if (bytes < 0) {
        char dbg_str[200], *s = dbg_str;
        ROS_ERROR("can_send_frame error %d %s",can_fd_,strerror(errno));
        sprintf(s,"%02x %02x %04x ",frame->_sof, frame->_dlc, read_le<uint16_t>((uint8_t*)&frame->_frame_id));
        s += 11;
        for (int i = 0 ; i <= data_len; i++) {
            sprintf(s,"%02x ",frame->_data[i]);
            s+=3;
        }
        *s= '\0';
        ROS_ERROR("packet %s",dbg_str);
    }
    //usleep(20);
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
    write_le<uint32_t>(state, cf_._data);
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
    write_le<uint32_t>(control_mode, cf_._data);
    write_le<uint32_t>(input_mode,   cf_._data + 4);
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
    write_le<float>(input_torque, cf_._data);
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
    write_le<float>(input_vel,    cf_._data);
    write_le<float>(input_torque, cf_._data + 4);
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
    write_le<float>(input_pos,  cf_._data);
    write_le<int8_t>(((int8_t)((input_vel) * 1000)), cf_._data + 4);
    write_le<int8_t>(((int8_t)((input_torque) * 1000)), cf_._data + 6);
    can_send_frame(can_id, CmdId::kSetInputPos, &cf_,8);
}

void odrive_can::can_set_absolute_position(int32_t can_id, float abs_pos) {
    can_frame cf_;
    memset(&cf_,0,sizeof(cf_));
    write_le<float>(abs_pos,  cf_._data);
    can_send_frame(can_id, CmdId::kSetAbsolutePos, &cf_,4);
}

void odrive_can::can_set_vel_gains(int32_t can_id, float vel_gain, float vel_integrator_gain) {
    can_frame cf_;
    memset(&cf_,0,sizeof(cf_));
    write_le<float>(vel_gain,cf_._data);
    write_le<float>(vel_integrator_gain,cf_._data+4);
    can_send_frame(can_id, CmdId::kSetVelGains, &cf_, 8);  
}