#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <robstride_driver.h>


uint8_t ROBSTRIDE_CAN_ID = 0x7B;
uint8_t ROBSTRIDE2_CAN_ID = 0x7C;
uint8_t CYBERGEAR_CAN_ID = 0x7D;
uint8_t MASTER_CAN_ID = 0x00;
char str;
RobStrideDriver robstride1 = RobStrideDriver(ROBSTRIDE_CAN_ID, MASTER_CAN_ID, MotorType::ROBSTRIDE03);
RobStrideDriver robstride2 = RobStrideDriver(ROBSTRIDE2_CAN_ID, MASTER_CAN_ID, MotorType::ROBSTRIDE00);
RobStrideDriver cybergear1 = RobStrideDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID, MotorType::CYBERGEAR);
RobStrideStatus motor_status1;
RobStrideStatus motor_status2;
RobStrideStatus motor_status3;
float position[3]{};
const float MAX_CURRENT = 10;
IntervalTimer myTimer;

//Specify the links and initial tuning parameters

CAN_message_t msg;

void Timtask(){
  static bool flag = false;
  
   static float pos = 0;
   static float vel =0;
  robstride1.request_status();
  robstride2.request_status();
  cybergear1.request_status();

   if(str == 'e') {
    Serial.print(" enable ");
    robstride1.enable_motor();
    robstride2.enable_motor();
    cybergear1.enable_motor();
    flag = true;

  }else if(str == 'd') {
    robstride1.disable_motor();
    robstride2.disable_motor();
    cybergear1.disable_motor();
    flag = false;
  } 


  else if( str == 'r'){
    robstride1.set_mechpos_zero();
    delay(1);
  }
  else if(str == 'i') {
    pos--;
    robstride1.set_position(pos);
    robstride2.set_position(pos);
    cybergear1.set_position(pos);
  } else if(str == 'o') {
    pos++;
    robstride1.set_position(pos);
    robstride2.set_position(pos);
    cybergear1.set_position(pos);
  } 

  motor_status1 = robstride1.get_status();
  motor_status2 = robstride2.get_status();
  motor_status3 = cybergear1.get_status();

  Serial.print(" Angle: "); Serial.print(motor_status1.position);
  // Serial.print(" Angular_vel: "); Serial.print(motor_status1.speed);
  // Serial.print(" Torque: "); Serial.print(motor_status1.torque);
  // Serial.print(" Tempareture: "); Serial.print(motor_status1.temperature);
  // Serial.print("  2: "); 
  // Serial.print(" Angle: "); Serial.println(motor_status2.position);
  // Serial.print(" Angular_vel: "); Serial.print(motor_status2.speed);
  // Serial.print(" Torque: "); Serial.println(motor_status2.torque);
  // Serial.print(" Tempareture: "); Serial.println(motor_status1.temperature);
  // Serial.print("  3: "); 
  // Serial.print(" Angle: "); Serial.println(motor_status3.position);
  // Serial.print(" Angular_vel: "); Serial.print(motor_status3.speed);
  // Serial.print(" Torque: "); Serial.println(motor_status3.torque);
  // Serial.print(" Tempareture: "); Serial.println(motor_status3.temperature);
  Serial.println();
  if(flag){
  }
  str=0;
}

void setup() {
  Serial.begin(115200); delay(400);
  // put your setup code here, to run once:
  robstride1.init_can();
  delay(1);
  robstride1.init_motor(MODE_POSITION);
  delay(1);
  robstride1.set_limit_speed(20.0f); /* set the maximum speed of the motor */
  delay(1);
  robstride1.set_limit_current(10.0); /* current limit allows faster operation */
  delay(1);
  robstride2.init_motor(MODE_POSITION);
  delay(1);
  robstride2.set_limit_speed(20.0f); /* set the maximum speed of the motor */
  delay(1);
  robstride2.set_limit_current(10.0); /* current limit allows faster operation */
  delay(1);
  cybergear1.init_motor(MODE_POSITION);
  delay(1);
  cybergear1.set_limit_speed(20.0f); /* set the maximum speed of the motor */
  delay(1);
  cybergear1.set_limit_current(10.0); /* current limit allows faster operation */
  delay(1);

  myTimer.begin(Timtask, 1000);

}

void loop() {

  if (Serial.available()) {
    str = Serial.read();
    Serial.print(str);
  }
}


