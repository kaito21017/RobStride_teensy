#ifndef ROBSTRIDE_DRIVER_H
#define ROBSTRIDE_DRIVER_H

#include <Arduino.h>  // for micros and delay function
#include "robstride_defs.h"
#include <FlexCAN_T4.h>

struct RobStrideStatus {
    uint8_t motor_can_id;
    float position;
    float pos_offset;
    float speed;
    float torque;
    uint16_t temperature;
};

struct MotorFault
{
  bool encoder_not_calibrated;
  bool over_current_phase_a;
  bool over_current_phase_b;
  bool over_voltage;
  bool under_voltage;
  bool driver_chip;
  bool motor_over_tempareture;
};


enum MotorType {
    CYBERGEAR,
    ROBSTRIDE00,
    ROBSTRIDE01,
    ROBSTRIDE02,
    ROBSTRIDE03,
    ROBSTRIDE04,
};

struct MotorParameter
{
  unsigned long stamp_usec;
  uint16_t run_mode;
  float iq_ref;
  float spd_ref;
  float limit_torque;
  float cur_kp;
  float cur_ki;
  float cur_filt_gain;
  float loc_ref;
  float limit_spd;
  float limit_cur;
  float mech_pos;
  float iqf;
  float mech_vel;
  float vbus;
  int16_t rotation;
  float loc_kp;
  float spd_kp;
  float spd_ki;
};


typedef struct RobStrideMotionCommand {
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
}RobStrideMotionCommand;

class RobStrideDriver {
    public:
        RobStrideDriver();
        RobStrideDriver(uint8_t master_can_id, uint8_t robstride_can_id, MotorType motorType);
        virtual ~RobStrideDriver();

        /**
         * @retval -1 Error
         * @retval 0 OK
         */
        int init_can();
        void init_motor(uint8_t mode);
        void enable_motor();
        void disable_motor();
        void set_run_mode(uint8_t mode);
        void set_mechpos_zero();

        void set_limit_speed(float speed);
        void set_limit_current(float current);
        void set_limit_torque(float torque);

        // MODE MOTION
        void send_motion_control(RobStrideMotionCommand cmd);

        // MODE_CURRENT
        void set_current_kp(float kp);
        void set_current_ki(float ki);
        void set_current_filter_gain(float gain);
        void set_current(float current);

        // MODE_POSITION
        void set_position_kp(float kp);
        void set_position(float position);
        void request_mech_pos();
        void get_mech_pos();

        // MODE_SPEED
        void set_speed_kp(float kp);
        void set_speed_ki(float ki);
        void set_speed(float speed);

        uint8_t get_run_mode() const;
        uint8_t get_motor_can_id() const;
        void set_motor_can_id(uint8_t can_id);

        void request_status();
        void process_message(CAN_message_t& message);
        void request_VBUS();
        void get_VBUS();
        RobStrideStatus get_status();
        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
        
    private:
        uint16_t _float_to_uint(float x, float x_min, float x_max, int bits);
        float _uint_to_float(uint16_t x, float x_min, float x_max);
        void _send_can_package(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data);
        void _send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min, float max);

        uint8_t _robstride_can_id;
        uint8_t _master_can_id;
        uint8_t _run_mode;
        bool _use_serial_debug;
        RobStrideStatus _status;
        MotorType motorType;
        const MotorParams* params;
        // FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
        
};

#endif