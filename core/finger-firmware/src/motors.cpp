#include "motors.hpp"

Motors::Motors() {
    this->numMotors = 0;
}


Motors::Motors(uint8_t numMotors, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf) {
    this->numMotors = numMotors;

    for(int i = 0; i < numMotors; i++) {
        (this->motorList).emplace_back(can_intf, i);

        // Register callbacks for the heartbeat and encoder feedback messages
        (this->motorList[i]).setFeedback();
        (this->motorList[i]).setStatus();
    }
}

void Motors::addMotor(Motor_Controller& motor) {
    this->motorList.push_back(motor);
    this->numMotors++;
}

float Motors::getMotorBusVoltage(uint8_t motorID) {
    return (this->motorList[motorID]).getBusVoltage();
}

float Motors::getMotorBusCurrent(uint8_t motorID) {
    return (this->motorList[motorID]).getBusCurrent();
}

void Motors::clearMotorErrors(uint8_t motorID) {
    (this->motorList[motorID]).clearErrors();
}

void Motors::setMotorState(uint8_t motorID, enum ODriveAxisState state) {
    (this->motorList[motorID]).setMotorState(state);
}

void Motors::setMotorPosition(uint8_t motorID, float position, float velocity_feedforward, float torque_feedforward) {
    (this->motorList[motorID]).setPosition(position, velocity_feedforward, torque_feedforward);
}

void Motors::setMotorVelocity(uint8_t motorID, float velocity, float torque_feedforward) {
    (this->motorList[motorID]).setVelocity(velocity, torque_feedforward);
}

void Motors::setMotorTorque(uint8_t motorID, float torque) {
    (this->motorList[motorID]).setTorque(torque);
}

float Motors::getMotorPosition(uint8_t motorID) {
    return (this->motorList[motorID]).getMotorPosition();
}

float Motors::getMotorVelocity(uint8_t motorID) {
    return (this->motorList[motorID]).getMotorVelocity();
}

uint8_t Motors::getMotorState(uint8_t motorID) {
    return (this->motorList[motorID]).getMotorState();
}

bool Motors::checkHeartbeat(uint8_t motorID) {
    return (this->motorList[motorID]).checkHeartbeat();
}

bool Motors::checkFeedback(uint8_t motorID) {
    return (this->motorList[motorID]).checkFeedback();
}

void Motors::setupOnReceive(const CanMsg& msg) {
    for(auto motor_controller : this->motorList) {
        onReceive(msg, *(motor_controller.getODrive()));
    }
}

void Motors::setTorque(std::vector<float> torque) {
    this->torque = torque;

    for(int i = 0; i < this-> numMotors; i++) {
        (this->motorList[i]).setTorque(torque[i]);
    }
}

std::vector<float> Motors::getTorque() {
    return this->torque;
}