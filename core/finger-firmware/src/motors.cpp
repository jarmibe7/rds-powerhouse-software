#include "motors.hpp"

Motors::Motors() {
    this->numMotors = 0;
}


Motors::Motors(uint8_t numMotors, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf) {
    this->numMotors = numMotors;

    for(int i = 0; i < numMotors; i++) {
        (this->motor_list).emplace_back(can_intf, i);

        // Register callbacks for the heartbeat and encoder feedback messages
        (this->motor_list[i]).setFeedback();
        (this->motor_list[i]).setStatus();
    }
}

void Motors::addMotor(Motor_Controller& motor) {
    this->motor_list.push_back(motor);
    this->numMotors++;
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

bool Motors::checkFeedback(uint8_t motorID) {
    return (this->motor_list[motorID]).checkFeedback();
}

void Motors::setupOnReceive(const CanMsg& msg) {
    for(auto motor_controller : this->motor_list) {
        onReceive(msg, *(motor_controller.getODrive()));
    }
}

