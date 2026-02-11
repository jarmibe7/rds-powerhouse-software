#include "motor_controller.hpp"

Motor_Controller::Motor_Controller(ODriveCAN* odrive, int node_id) {
    this->nodeId = node_id;
    this->odrive = odrive;
}

void Motor_Controller::setFeedback(void (*callback)(Get_Encoder_Estimates_msg_t& feedback, void* user_data), void* user_data) {
    this->odrive->onFeedback(callback, user_data);
}

void Motor_Controller::setStatus(void (*callback)(Heartbeat_msg_t& feedback, void* user_data), void* user_data = nullptr) {
    this->odrive->onStatus(callback, user_data);
}

float Motor_Controller::getBusVoltage() {
    Serial.println("starting voltage read");
    if (!((this->odrive)->request(this->vbus, 1000))) {
        Serial.println("vbus request failed!");
    }
    Serial.println("Made it here");
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



Motors::Motors() {
    this->numMotors = 0;
}

void Motors::addMotor(Motor_Controller& motor) {
    this->motor_list.push_back(motor);
    this->numMotors++;
}

void Motors::setFeedback(uint8_t motorID, void (*callback)(Get_Encoder_Estimates_msg_t& feedback, void* user_data), void* user_data) {
    (this->motor_list[motorID]).setFeedback(callback, user_data);
}

void Motors::setStatus(uint8_t motorID, void (*callback)(Heartbeat_msg_t& feedback, void* user_data), void* user_data = nullptr) {
    (this->motor_list[motorID]).setStatus(callback, user_data);
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
