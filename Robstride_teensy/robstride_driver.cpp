#include "robstride_driver.h"

/* PUBLIC */
RobStrideDriver::RobStrideDriver() {};
RobStrideDriver::RobStrideDriver(uint8_t robstride_can_id, uint8_t master_can_id, MotorType motorType) 
    : _robstride_can_id(robstride_can_id),
    _master_can_id(master_can_id),
    _run_mode(MODE_MOTION),
    motorType(motorType),
    _use_serial_debug(false)
    
{
    switch (motorType) {
            case CYBERGEAR:
                params = &params_CyberGear;
                break;
            case ROBSTRIDE00:
                params = &params_RobStride00;
                break;
            case ROBSTRIDE01:
                params = &params_RobStride01;
                break;
            case ROBSTRIDE02:
                params = &params_RobStride02;
                break;
            case ROBSTRIDE03:
                params = &params_RobStride03;
                break;
            case ROBSTRIDE04:
                params = &params_RobStride04;
                break;
            default:
                params = nullptr; // エラーハンドリング用
                break;
        }
}

RobStrideDriver::~RobStrideDriver(){}

int RobStrideDriver::init_can(){
    // FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
    can1.begin();
    can1.setBaudRate(1000000);
    return 0;
}

void RobStrideDriver::init_motor(uint8_t mode){
    disable_motor();
    set_run_mode(mode);
}
void RobStrideDriver::enable_motor(){
    uint8_t data[8] = {0x00};
    _send_can_package(_robstride_can_id, CMD_ENABLE, _master_can_id, 8, data);
}
void RobStrideDriver::disable_motor(){
    uint8_t data[8] = {0x00};
    _send_can_package(_robstride_can_id, CMD_STOP, _master_can_id, 8, data);
}
void RobStrideDriver::set_run_mode(uint8_t mode){
    _run_mode = mode;
    uint8_t data[8] = {0x00};
    data[0] = ADDR_RUN_MODE & 0x00FF;
    data[1] = ADDR_RUN_MODE >> 8;
    data[4] = mode;
    _send_can_package(_robstride_can_id, CMD_RAM_WRITE, _master_can_id, 8, data);
}

void RobStrideDriver::set_mechpos_zero(){
    uint8_t data[8] = {0x00};
    data[0] = 1;
    _send_can_package(_robstride_can_id, CMD_SET_MECH_POSITION_TO_ZERO, _master_can_id, 8, data);
}

void RobStrideDriver::set_limit_speed(float speed){
    _send_can_float_package(_robstride_can_id, ADDR_LIMIT_SPEED, speed, 0.0f, params->V_MAX);
}
void RobStrideDriver::set_limit_current(float current){
    _send_can_float_package(_robstride_can_id, ADDR_LIMIT_CURRENT, current, 0.0f, params->IQ_MAX);
}
void RobStrideDriver::set_limit_torque(float torque){
    _send_can_float_package(_robstride_can_id, ADDR_LIMIT_TORQUE, torque, 0.0f, params->T_MAX);
}

// MODE_MOTION
void RobStrideDriver::send_motion_control(RobStrideMotionCommand cmd){
    uint8_t data[8] = {0x00};

    uint16_t position = _float_to_uint(cmd.position, params->P_MIN, params->P_MAX, 16);
    data[0] = position >> 8;
    data[1] = position & 0x00FF;

    uint16_t speed = _float_to_uint(cmd.speed, params->V_MIN, params->V_MAX, 16);
    data[2] = speed >> 8;
    data[3] = speed & 0x00FF;

    uint16_t kp = _float_to_uint(cmd.kp, KP_MIN, KP_MAX, 16);
    data[4] = kp >> 8;
    data[5] = kp & 0x00FF;

    uint16_t kd = _float_to_uint(cmd.kd, KD_MIN, KD_MAX, 16);
    data[6] = kd >> 8;
    data[7] = kd & 0x00FF;

    uint16_t torque = _float_to_uint(cmd.torque, params->T_MIN, params->T_MAX, 16);

    _send_can_package(_robstride_can_id, CMD_POSITION, torque, 8, data);
}

// MODE_CURRENT
void RobStrideDriver::set_current_kp(float kp){
    _send_can_float_package(_robstride_can_id, ADDR_CURRENT_KP, kp, KP_MIN, KP_MAX);
}
void RobStrideDriver::set_current_ki(float ki){
    _send_can_float_package(_robstride_can_id, ADDR_CURRENT_KI, ki, KI_MIN, KI_MAX);
}
void RobStrideDriver::set_current_filter_gain(float gain){
    _send_can_float_package(_robstride_can_id, ADDR_CURRENT_FILTER_GAIN, gain, CURRENT_FILTER_GAIN_MIN, CURRENT_FILTER_GAIN_MAX);
}
void RobStrideDriver::set_current(float current){
    _send_can_float_package(_robstride_can_id, ADDR_IQ_REF, current, params->IQ_MIN, params->IQ_MAX);
}

// MODE_POSITION
void RobStrideDriver::set_position_kp(float kp){
    _send_can_float_package(_robstride_can_id, ADDR_POSITION_KP, kp, KP_MIN, KP_MAX);
}
void RobStrideDriver::set_position(float position){
    _send_can_float_package(_robstride_can_id, ADDR_POSITION_REF, position, params->P_MIN, params->P_MAX);
}

// MODE_SPEED
void RobStrideDriver::set_speed_kp(float kp){
    _send_can_float_package(_robstride_can_id, ADDR_SPEED_KP, kp, KP_MIN, KP_MAX);
}
void RobStrideDriver::set_speed_ki(float ki){
    _send_can_float_package(_robstride_can_id, ADDR_SPEED_KI, ki, KI_MIN, KI_MAX);
}
void RobStrideDriver::set_speed(float speed){
    _send_can_float_package(_robstride_can_id, ADDR_SPEED_REF, speed, params->V_MIN, params->V_MAX);
}

void RobStrideDriver::set_motor_can_id(uint8_t can_id){
    uint8_t data[8] = {0x00};
    uint16_t option = can_id << 8 | _master_can_id;
    _send_can_package(_robstride_can_id, CMD_SET_CAN_ID, option, 8, data);
    _robstride_can_id = can_id;
}

uint8_t RobStrideDriver::get_run_mode() const {
    return _run_mode;
}
uint8_t RobStrideDriver::get_motor_can_id() const {
    return _robstride_can_id;
}

void RobStrideDriver::request_status() {
    uint8_t data[8] = {0x00};
    _send_can_package(_robstride_can_id, CMD_GET_STATUS, _master_can_id, 8, data);
}
void RobStrideDriver::process_message(CAN_message_t& message){
    uint16_t raw_position = message.buf[1] | message.buf[0] << 8;
    uint16_t raw_speed = message.buf[3] | message.buf[2] << 8;
    uint16_t raw_torque = message.buf[5] | message.buf[4] << 8;
    uint16_t raw_temperature = message.buf[7] | message.buf[6] << 8;

    _status.position = _uint_to_float(raw_position, params->P_MIN, params->P_MAX);
    _status.speed = _uint_to_float(raw_speed, params->V_MIN, params->V_MAX);
    _status.torque = _uint_to_float(raw_torque, params->T_MIN, params->T_MAX);
    _status.temperature = raw_temperature;
}
RobStrideStatus RobStrideDriver::get_status() {
    CAN_message_t r_message;
    if(can1.read(r_message)){
        if (((r_message.id & 0xFF00) >> 8) == _robstride_can_id){
            uint16_t raw_position = r_message.buf[1] | r_message.buf[0] << 8;
            uint16_t raw_speed = r_message.buf[3] | r_message.buf[2] << 8;
            uint16_t raw_torque = r_message.buf[5] | r_message.buf[4] << 8;
            uint16_t raw_temperature = r_message.buf[7] | r_message.buf[6] << 8;

            _status.position = _uint_to_float(raw_position, params->P_MIN, params->P_MAX);
            _status.speed = _uint_to_float(raw_speed, params->V_MIN, params->V_MAX);
            _status.torque = _uint_to_float(raw_torque, params->T_MIN, params->T_MAX);
            _status.temperature = raw_temperature/10;
        }
    }


    return _status;
}



/* PRIVATE */
uint16_t RobStrideDriver::_float_to_uint(float x, float x_min, float x_max, int bits){
    if (bits>16) bits=16;
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
float RobStrideDriver::_uint_to_float(uint16_t x, float x_min, float x_max){
    uint16_t type_max = 0xFFFF;
    float span = x_max - x_min;
    return (float) x / type_max * span + x_min;
}
void RobStrideDriver::_send_can_package(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data){
    uint32_t id = cmd_id << 24 | option << 8 | can_id;

    CAN_message_t message;
    message.flags.extended = id;
    message.id = id;
    message.len = len;
    for (int i = 0; i < len; i++) {
        message.buf[i] = data[i];
    }

    // Queue message for transmission
    // if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    //     // if (_use_serial_debug) Serial.println("Message queued for transmission\n");
    // } else {
    //     if (_use_serial_debug) Serial.println("Failed to queue message for transmission\n");
    // }
    can1.write(message);
}

void RobStrideDriver::_send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min, float max){
    uint8_t data[8] = {0x00};
    data[0] = addr & 0x00FF;
    data[1] = addr >> 8;

    float val = (max < value) ? max : value;
    val = (min > value) ? min : value;
    memcpy(&data[4], &value, 4);
    _send_can_package(can_id, CMD_RAM_WRITE, _master_can_id, 8, data);
}