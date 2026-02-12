#include "motor_controller.hpp"

Motor_Controller::Motor_Controller(ODriveCAN* odrive, int node_id) {
    this->nodeId = node_id;
    this->odrive = odrive;
}

void Motor_Controller::setFeedback(void (*callback)(Get_Encoder_Estimates_msg_t& feedback, void* user_data)) {
    this->odrive->onFeedback(callback, &(this->user_data));
}

void Motor_Controller::setStatus(void (*callback)(Heartbeat_msg_t& feedback, void* user_data)) {
    this->odrive->onStatus(callback, &(this->user_data));
}

float Motor_Controller::getBusVoltage() {
    Serial.println("starting voltage read");
    if (!((this->odrive)->request(this->vbus, 1000))) {
        Serial.println("vbus request failed!");
    }
    return this->vbus.Bus_Voltage;
}
float Motor_Controller::getBusCurrent() {
    if (!((this->odrive)->request(this->vbus, 1000))) {
        Serial.println("vbus request failed!");
    }
    return this->vbus.Bus_Voltage;
}

void Motor_Controller::clearErrors() {
    this->odrive->clearErrors();
}

void Motor_Controller::setMotorState(enum ODriveAxisState state) {
    this->odrive->setState(state);
}

void Motor_Controller::setPosition(float position, float velocity_feedforward, float torque_feedforward) {
    this->odrive->setPosition(position, velocity_feedforward, torque_feedforward);
}

void Motor_Controller::setVelocity(float velocity, float torque_feedforward) {
    this->odrive->setVelocity(velocity, torque_feedforward);
}

void Motor_Controller::setTorque(float torque) {
    this->odrive->setTorque(torque);
}

float Motor_Controller::getMotorPosition() {
    return this->user_data.last_feedback.Pos_Estimate;
}

float Motor_Controller::getMotorVelocity() {
    return this->user_data.last_feedback.Pos_Estimate;
}

uint8_t Motor_Controller::getMotorState() {
    return this->user_data.last_heartbeat.Axis_State;
}

bool Motor_Controller::checkHeartbeat() {
    bool heartbeat = this->user_data.received_heartbeat;
    this->user_data.received_heartbeat = false;
    return heartbeat;
}

ODriveCAN* Motor_Controller::getODrive() {
    return this->odrive;
}




Motors::Motors() {
    this->numMotors = 0;
}

void Motors::addMotor(Motor_Controller& motor) {
    this->motor_list.push_back(motor);
    this->numMotors++;
}

void Motors::setFeedback(uint8_t motorID, void (*callback)(Get_Encoder_Estimates_msg_t& feedback, void* user_data)) {
    (this->motor_list[motorID]).setFeedback(callback);
}

void Motors::setStatus(uint8_t motorID, void (*callback)(Heartbeat_msg_t& feedback, void* user_data)) {
    (this->motor_list[motorID]).setStatus(callback);
}

float Motors::getMotorBusVoltage(uint8_t motorID) {
    return (this->motor_list[motorID]).getBusVoltage();
}

float Motors::getMotorBusCurrent(uint8_t motorID) {
    return (this->motor_list[motorID]).getBusCurrent();
}

void Motors::clearMotorErrors(uint8_t motorID) {
    (this->motor_list[motorID]).clearErrors();
}

void Motors::setMotorState(uint8_t motorID, enum ODriveAxisState state) {
    (this->motor_list[motorID]).setMotorState(state);
}

void Motors::setMotorPosition(uint8_t motorID, float position, float velocity_feedforward, float torque_feedforward) {
    (this->motor_list[motorID]).setPosition(position, velocity_feedforward, torque_feedforward);
}

void Motors::setMotorVelocity(uint8_t motorID, float velocity, float torque_feedforward) {
    (this->motor_list[motorID]).setVelocity(velocity, torque_feedforward);
}

void Motors::setMotorTorque(uint8_t motorID, float torque) {
    (this->motor_list[motorID]).setTorque(torque);
}

float Motors::getMotorPosition(uint8_t motorID) {
    return (this->motor_list[motorID]).getMotorPosition();
}

float Motors::getMotorVelocity(uint8_t motorID) {
    return (this->motor_list[motorID]).getMotorVelocity();
}

uint8_t Motors::getMotorState(uint8_t motorID) {
    return (this->motor_list[motorID]).getMotorState();
}

bool Motors::checkHeartbeat(uint8_t motorID) {
    return (this->motor_list[motorID]).checkHeartbeat();
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