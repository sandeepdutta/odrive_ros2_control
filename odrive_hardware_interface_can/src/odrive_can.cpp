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

    tty_fd = open(tty, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (tty_fd == -1) {
        ROS_ERROR("open(%s) failed: %s", tty, strerror(errno));
        return -1;
    }

    result = ioctl(tty_fd, TCGETS2, &tio);
    if (result == -1) {
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
    if (result == -1) {
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
void odrive_can::can_thread() {
    fd_set can_set;
    FD_ZERO(&can_set);
    FD_SET(can_fd_, &can_set);
    while (1) {
        int32_t can_id, cmd_id;
        can_frame can_frame_ ;
        if (select(1, &can_set, NULL, NULL, NULL ) < 0) {
            ROS_ERROR("select error terminating %s",strerror(errno));
            break;
        }
        // file description ready to read
        int frame_len = can_recv_frame(can_id, cmd_id, (uint8_t *)&can_frame_);
        if (frame_len < 0) ROS_ERROR("frame receive error");
        // if we cannot determine the frame then continue
        if (cmd_id == -1) continue;
        int axis = (*canid_axis_)[can_id];
        switch(cmd_id) {
        case CmdId::kHeartbeat: {
            if (!verify_length("kHeartbeat", 8, frame_len-5)) break;
            hw_atomics_->can_shared_active_errors       [axis] = read_le<uint32_t>(can_frame_._data + 0);
            hw_atomics_->can_shared_axis_state          [axis] = read_le<uint8_t>(can_frame_._data + 4);
            hw_atomics_->can_shared_procedure_result    [axis] = read_le<uint8_t>(can_frame_._data + 5);
            hw_atomics_->can_shared_trajectory_done_flag[axis] = read_le<bool>(can_frame_._data + 6);
            break;
        }
        /*
        case CmdId::kGetError: {
            if (!verify_length("kGetError", 8, frame.len)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
            odrv_pub_flag_ |= 0b001;
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            if (!verify_length("kGetEncoderEstimates", 8, frame.len)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
            ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0010;
            break;
        }
        case CmdId::kGetIq: {
            if (!verify_length("kGetIq", 8, frame.len)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0100;
            break;
        }
        case CmdId::kGetTemp: {
            if (!verify_length("kGetTemp", 8, frame.len)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.fet_temperature   = read_le<float>(frame.data + 0);
            odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b010;
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            if (!verify_length("kGetBusVoltageCurrent", 8, frame.len)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
            odrv_stat_.bus_current = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b100;
            break;
        }
        case CmdId::kGetTorques: {
            if (!verify_length("kGetTorques", 8, frame.len)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.torque_target   = read_le<float>(frame.data + 0);
            ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b1000; 
            break;
        }*/
        default: {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "Received unused message: ID = 0x%x", (can_id & 0x1F));
            break;
        }
        }
    }
}

// set commands
void odrive_can::can_request_state(int32_t can_id, uint32_t state) {

}
void odrive_can::can_set_control_mode(int32_t can_id, uint32_t control_mode , uint32_t input_mode ) {

}
void odrive_can::can_set_input_torque(int32_t can_id, float input_torque) {

}
void odrive_can::can_set_input_vel_torque(int32_t can_id, float input_vel, float input_torque) {

}
void odrive_can::can_set_position(int32_t can_id, float input_pos, uint8_t input_vel, uint8_t input_torque) {

}