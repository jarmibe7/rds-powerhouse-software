#include "motor.hpp"


// Motor::Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int node_id) : odrive(wrap_can_intf(can_intf), node_id) {
    
//     // ODriveCAN odrive(wrap_can_intf(can_intf), node_id); // Standard CAN message ID
//     this->nodeId = node_id;
// }

Motor::Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int motor_id) : can_intf(can_intf), motorId(motor_id) {}

void Motor::setTorque(float torque) {
    int32_t torque_mA = (int32_t) (1000 * torque /(GEAR_RATIO * MOTOR_KT));
    sendCanMsgInt(this->can_intf, (0x1 << 8) | this->motorId, true, 4, torque_mA);
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

void Motor::setup() {
    // Serial.println("Setting limits");
    // this->odrive.setLimits(300.0, 5.0);
    // Serial.println("Limits set");  
    // this->odrive.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
}