#include "motor.hpp"


// Motor::Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int node_id) : odrive(wrap_can_intf(can_intf), node_id) {
    
//     // ODriveCAN odrive(wrap_can_intf(can_intf), node_id); // Standard CAN message ID
//     this->nodeId = node_id;
// }

Motor::Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int motor_id) : can_intf(can_intf), motorId(motor_id) {}

void Motor::setup() {
    enterMotorControlMode(this->can_intf, this->motorId);
}

void Motor::setTorque(float torque) {
    // int32_t torque_mA = (int32_t) (1000 * torque /(GEAR_RATIO * MOTOR_KT));
    // sendCanMsgInt(this->can_intf, (0x1 << 8) | this->motorId, true, 4, torque_mA);
    float torque_A = (torque /(GEAR_RATIO * MOTOR_KT));
    // sendCanMsgInt(this->can_intf, (0x1 << 8) | this->motorId, true, 4, torque_mA);


    pack_cmd(this->can_intf, this->motorId, 0, 0, 0, 0, torque_A);
    
}

float Motor::getMotorPosition() {
    return this->position;
}

float Motor::getMotorVelocity() {
    return this->velocity;
}

float Motor::getMotorCurrent() {
    return this->current;
}

float Motor::getMotorTemperature() {
    return this->temperature;
}

uint8_t Motor::getMotorError() {
    return this->error;
}

void Motor::updateFeedback(float position, float velocity, float current, float temperature, uint8_t error) {
    this->position = position;
    this->velocity = velocity;
    this->current = current;
    this->temperature = temperature;
    this->error = error;
}

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}




void pack_cmd(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, uint8_t motorID, float p_des, float v_des, float kp, float kd, float t_ff) {
  float P_MIN =-12.5f;
  float P_MAX =12.5f;
  float V_MIN =-45.5f;
  float V_MAX =45.5f;
  float T_MIN =-5.0f;
  float T_MAX =5.0f;
  float Kp_MIN =0;
  float Kp_MAX =500.0f;
  float Kd_MIN =0;
  float Kd_MAX =5.0f;
  float Test_Pos=0.0f;
  int p_int ;
  int v_int;
  int kp_int ;
  int kd_int;
  int t_int ;
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
  p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  v_int= float_to_uint(v_des, V_MIN, V_MAX, 12);
  kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
  kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
  t_int= float_to_uint(t_ff, T_MIN, T_MAX, 12);


    uint8_t buf[8];

  buf[0] = p_int >> 8; // Position high 8 bits
  buf[1] = p_int & 0xFF; // Position low 8 bits
  buf[2] = v_int >> 4; // Speed high 8 bits
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed low 4 bits, KP high 4 bits
  buf[4] = kp_int & 0xFF; // KP low 8 bits
  buf[5] = kd_int >> 4; // Kd high 8 bits
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // Kd low 4 bits, torque high 4 bits
  buf[7] = t_int & 0xFF; // Torque low 8 bits

//   Serial.printf("%x %x\n", msg.buf[6], msg.buf[7]);

//   msg.id = 2;
//   msg.len = 8;
//   msg.flags.extended = 0;
  
//   can1.write(msg);

  sendCanBuffer(can_intf, motorID, false, 8, buf);
}


void enterMotorControlMode(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, uint8_t motorID) {

    uint8_t buf[8];
  
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFC;


  sendCanBuffer(can_intf, motorID, false, 8, buf);

  delay(10);
}


int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  if(x < x_min) x = x_min;
  else if(x > x_max) x = x_max;
  return (int) ((x- x_min)*((float)((1<<bits)/span)));
}