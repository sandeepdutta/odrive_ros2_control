#ifndef ODRIVE_CAN_HPP_
#define ODRIVE_CAN_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <endian.h>
#include <mutex>
#include <thread>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <asm/termbits.h> /* struct termios2 */
#include <sys/select.h>
#include <time.h>
#include <ctype.h>
#include <signal.h>
#include <sys/time.h>
#include "rclcpp/rclcpp.hpp"
#include <odrive_enums.hpp>
#include "byte_swap.hpp"
#define CANUSB_INJECT_SLEEP_GAP_DEFAULT 200 /* ms */
#define CANUSB_TTY_BAUD_RATE_DEFAULT 2000000
#define MAX_AXIS    2
#define MAX_SENSORS 1
#define MAX_CAN_FRAME_SIZE 32

typedef enum {
  CANUSB_SPEED_1000000 = 0x01,
  CANUSB_SPEED_800000  = 0x02,
  CANUSB_SPEED_500000  = 0x03,
  CANUSB_SPEED_400000  = 0x04,
  CANUSB_SPEED_250000  = 0x05,
  CANUSB_SPEED_200000  = 0x06,
  CANUSB_SPEED_125000  = 0x07,
  CANUSB_SPEED_100000  = 0x08,
  CANUSB_SPEED_50000   = 0x09,
  CANUSB_SPEED_20000   = 0x0a,
  CANUSB_SPEED_10000   = 0x0b,
  CANUSB_SPEED_5000    = 0x0c,
} CANUSB_SPEED;

typedef enum {
  CANUSB_MODE_NORMAL          = 0x00,
  CANUSB_MODE_LOOPBACK        = 0x01,
  CANUSB_MODE_SILENT          = 0x02,
  CANUSB_MODE_LOOPBACK_SILENT = 0x03,
} CANUSB_MODE;

typedef enum {
  CANUSB_FRAME_STANDARD = 0x01,
  CANUSB_FRAME_EXTENDED = 0x02,
} CANUSB_FRAME;

typedef enum {
    CANUSB_INJECT_PAYLOAD_MODE_RANDOM      = 0,
    CANUSB_INJECT_PAYLOAD_MODE_INCREMENTAL = 1,
    CANUSB_INJECT_PAYLOAD_MODE_FIXED       = 2,
} CANUSB_PAYLOAD_MODE;

typedef struct atomic_variables {
    std::atomic<double>   can_shared_vbus_voltages_       [MAX_AXIS] ;
    std::atomic<double>   can_shared_vbus_currents_       [MAX_AXIS] ;
    std::atomic<double>   can_shared_positions_           [MAX_AXIS] ;
    std::atomic<double>   can_shared_velocities_          [MAX_AXIS] ;
    std::atomic<double>   can_shared_efforts_             [MAX_AXIS] ;
    std::atomic<double>   can_shared_torque_targets_      [MAX_AXIS] ;
    std::atomic<double>   can_shared_axis_errors_         [MAX_AXIS] ;
    std::atomic<double>   can_shared_motor_temperatures_  [MAX_AXIS] ;
    std::atomic<double>   can_shared_fet_temperatures_    [MAX_AXIS] ;
    std::atomic<double>   can_shared_motor_currents_      [MAX_AXIS] ;
    std::atomic<uint32_t> can_shared_active_errors_       [MAX_AXIS];
    std::atomic<uint8_t>  can_shared_axis_state_          [MAX_AXIS];
    std::atomic<uint8_t>  can_shared_procedure_result_    [MAX_AXIS];
    std::atomic<bool>     can_shared_trajectory_done_flag_[MAX_AXIS]; 
    std::atomic<uint32_t> can_shared_disarm_reason_       [MAX_AXIS];
} atomic_variables;

typedef struct can_frame {
    uint8_t     _sof;     /* */
    uint8_t     _dlc ;      /* data length code */
    uint16_t    _frame_id;  /* */
    uint8_t     _data[MAX_CAN_FRAME_SIZE];
} can_frame;

class odrive_can {
private:
    int can_fd_;
    atomic_variables *hw_atomics_;
    std::map<int32_t, int32_t> *canid_axis_;
    int can_adapter_init(const char *tty, int baud, int can_speed);
    // can send frame 
    int  can_send_frame(uint32_t can_id, uint32_t cmd_id, can_frame *data, int32_t len);    
    // check if frame being received is complete
    bool can_frame_complete(const uint8_t *frame, int frame_len);
    // receive a frame
    int can_recv_frame(int32_t &can_id, int32_t &cmd_id, uint8_t *frame );
public:
    odrive_can(std::string *can_tty, int can_speed, atomic_variables *hw_atomics, std::map<int32_t,int32_t> *canid_axis) {
        hw_atomics_ = hw_atomics;
        canid_axis_ = canid_axis;
        if ((can_fd_ = can_adapter_init(can_tty->c_str(), CANUSB_TTY_BAUD_RATE_DEFAULT, can_speed)) < 0) {
            return;
        }
    }
    ~odrive_can() {
        close(can_fd_);
    }
    // set commands
    void can_request_state(int32_t can_id, uint32_t state);
    void can_set_control_mode(int32_t can_id, uint32_t control_mode , uint32_t input_mode );
    void can_set_input_torque(int32_t can_id, float input_torque);
    void can_set_input_vel_torque(int32_t can_id, float input_vel, float input_torque);
    void can_set_position(int32_t can_id, float input_pos, uint8_t input_vel, uint8_t input_torque);
    // main thread for the can controller
    void can_thread() ; 
    inline bool verify_length(const std::string&name, uint8_t expected, uint8_t length) {
        bool valid = expected == length;
        RCLCPP_DEBUG(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "received %s", name.c_str());
        if (!valid) RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
        return valid;
    }
};


#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterfaceCAN"),__VA_ARGS__)
#define ROS_INFO(...)  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterfaceCAN"),__VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("ODriveHardwareInterfaceCAN"),__VA_ARGS__)



#endif // ODRIVE_CAN_HPP_
