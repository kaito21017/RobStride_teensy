#ifndef ROBSTRIDE_DEFS_H
#define ROBSTRIDE_DEFS_H

#define CMD_POSITION 1
#define CMD_RESPONSE 2
#define CMD_ENABLE 3
#define CMD_STOP 4
#define CMD_SET_MECH_POSITION_TO_ZERO 6
#define CMD_SET_CAN_ID 7
#define CMD_GET_STATUS 15
#define CMD_RAM_READ 17
#define CMD_RAM_WRITE 18
#define CMD_GET_MOTOR_FAIL 21

#define ADDR_SPEED_KP 0x2004
#define ADDR_MECH_OFFSET 0x2005
#define ADDR_CHASU_OFFSET 0x2006
#define ADDR_ELC_OFFSET 0x2007
#define ADDR_I_FW_MAX 0x2008
#define ADDR_CAN_ID 0x2009
#define ADDR_CAN_MASTER 0x200a
#define ADDR_CAN_TIMEOUT 0x200b
#define ADDR_MOTER_OVER_TMP 0x200c
#define ADDR_OVER_TIME_TEMP 0x200d
#define ADDR_GEAR_RATIO 0x200e
#define ADDR_SPEED_KP 0x2014
#define ADDR_SPEED_KI 0x2015
#define ADDR_POSITION_KP 0x2016
#define ADDR_SPEED_FILT_GAIN 0x2017
#define ADDR_LIMIT_SPD 0x2018
#define ADDR_LIMIT_CUR 0x2019

#define ADDR_RUN_MODE 0x7005
#define ADDR_IQ_REF 0x7006
#define ADDR_SPEED_REF 0x700A
#define ADDR_LIMIT_TORQUE 0x700B
#define ADDR_CURRENT_KP 0x7010
#define ADDR_CURRENT_KI 0x7011
#define ADDR_CURRENT_FILTER_GAIN 0x7014
#define ADDR_POSITION_REF          0x7016
#define ADDR_LIMIT_SPEED 0x7017
#define ADDR_LIMIT_CURRENT 0x7018
#define ADDR_MECH_POS 0x7019
#define ADDR_IQF 0x701A
#define ADDR_MECH_VEL 0x701B
#define ADDR_VBUS 0x701C
#define ADDR_ROTATION 0x701D
#define ADDR_POSITION_KP 0x701E
#define ADDR_SPEED_KP 0x701F
#define ADDR_SPEED_KI 0x7020
#define ADDR_SPEED_FILT_GAIN 0x7021
#define ADDR_ACC_RAD 0x7022
#define ADDR_VEL_MAX 0x7024
#define ADDR_ACC_SET 0x7025

#define ADDR_ROTATION 0x701D
#define ADDR_POSITION_KP 0x701E
#define ADDR_SPEED_KP 0x701F
#define ADDR_SPEED_KI 0x7020
#define ADDR_SPEED_FILT_GAIN 0x7021
#define ADDR_ACC_RAD 0x7022
#define ADDR_VEL_MAX 0x7024
#define ADDR_ACC_SET 0x7025

#define MODE_MOTION 0x00
#define MODE_POSITION 0x01
#define MODE_SPEED 0x02
#define MODE_CURRENT 0x03

// #define P_MIN -12.5f
// #define P_MAX 12.5f
// #define V_MIN -30.0f
// #define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KI_MIN                      0.0f
#define KI_MAX                     10.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
// #define T_MIN -12.0f
// #define T_MAX 12.0f
// #define IQ_MIN -27.0f
// #define IQ_MAX 27.0f
#define CURRENT_FILTER_GAIN_MIN 0.0f
#define CURRENT_FILTER_GAIN_MAX 1.0f

// #define IQ_REF_MAX 23.0f
// #define IQ_REF_MIN -23.0f
// #define SPD_REF_MAX 30.0f
// #define SPD_REF_MIN -30.0f
// #define LIMIT_TORQUE_MAX 12.0f
// #define LIMIT_TORQUE_MIN 0.0f
// #define CUR_KP_MAX 200.0f
// #define CUR_KP_MIN 0.0f
// #define CUR_KI_MAX 200.0f
// #define CUR_KI_MIN 0.0f
// #define LOC_KP_MAX 200.0f
// #define LOC_KP_MIN 0.0f
// #define SPD_KP_MAX 200.0f
// #define SPD_KP_MIN 0.0f
// #define LIMIT_SPD_MAX 30.0f
// #define LIMIT_SPD_MIN 0.0f
// #define LIMIT_CURRENT_MAX 27.0f
// #define LIMIT_CURRENT_MIN 0.0f

// #define DEFAULT_CURRENT_KP 0.125f
// #define DEFAULT_CURRENT_KI 0.0158f
// #define DEFAULT_CURRENT_FINTER_GAIN 0.1f
// #define DEFAULT_POSITION_KP 30.0f
// #define DEFAULT_VELOCITY_KP 2.0f
// #define DEFAULT_VELOCITY_KI 0.002f
// #define DEFAULT_VELOCITY_LIMIT 2.0f
// #define DEFAULT_CURRENT_LIMIT 27.0f
// #define DEFAULT_TORQUE_LIMIT 12.0f

#define RET_ROBSTRIDE_OK 0x00
#define RET_ROBSTRIDE_MSG_NOT_AVAIL 0x01
#define RET_ROBSTRIDE_INVALID_CAN_ID 0x02
#define RET_ROBSTRIDE_INVALID_PACKET 0x03

#define ROBSTRIDE_RESPONSE_TIME_USEC 250

#define CW 1
#define CCW -1



#define pi 3.1415926

#define CW 1
#define CCW -1

struct MotorParams {
  float P_MIN;
  float P_MAX;
  float V_MIN;
  float V_MAX;
  // float KP_MIN;
  // float KP_MAX;
  // float KI_MIN;
  // float KI_MAX;
  // float KD_MIN;
  // float KD_MAX;
  float T_MIN;
  float T_MAX;
  float IQ_MIN;
  float IQ_MAX ;
  // float CURRENT_FILTER_GAIN_MIN;
  // float CURRENT_FILTER_GAIN_MAX;
};

const MotorParams params_CyberGear = {
    -4.f*pi, 4.f*pi,  // P_MIN, P_MAX
    -30.0f, 30.0f,  // V_MIN, V_MAX
    -12.0f, 12.0f,     // T_MIN, T_MAX
    -23.f, 23.f  //IQ_MIN, IQ_MAX
};
const MotorParams params_RobStride00 = {
    -4.f*pi, 4.f*pi,  // P_MIN, P_MAX
    -33.0f, 33.0f,  // V_MIN, V_MAX
    -14.0f, 14.0f,     // T_MIN, T_MAX
    -15.5f, 15.5f  //IQ_MIN, IQ_MAX
};
const MotorParams params_RobStride01 = {
    -4.f*pi, 4.f*pi,  // P_MIN, P_MAX
    -44.0f, 44.0f,  // V_MIN, V_MAX
    -17.0f, 17.0f,     // T_MIN, T_MAX
    -23.f, 23.f  //IQ_MIN, IQ_MAX
};
const MotorParams params_RobStride02 = {
    -4.f*pi, 4.f*pi,  // P_MIN, P_MAX
    -44.0f, 44.0f,  // V_MIN, V_MAX
    -17.0f, 17.0f,     // T_MIN, T_MAX
    -23.f, 23.f  //IQ_MIN, IQ_MAX
};
const MotorParams params_RobStride03 = {
    -4.f*pi, 4.f*pi,  // P_MIN, P_MAX
    -20.0f, 20.0f,  // V_MIN, V_MAX
    -60.0f, 60.0f,     // T_MIN, T_MAX
    -43.f, 43.f  //IQ_MIN, IQ_MAX
};
const MotorParams params_RobStride04 = {
    -4.f*pi, 4.f*pi,  // P_MIN, P_MAX
    -15.0f, 15.0f,  // V_MIN, V_MAX
    -120.0f, 120.0f,     // T_MIN, T_MAX
    -90.f, 90.f  //IQ_MIN, IQ_MAX
};


#endif  // !ROBSTRIDE_GEAR_DRIVER_DEFS_H
