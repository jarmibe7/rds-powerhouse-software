#include "motor_controller.hpp"

MotorController::MotorController() {
    this->numMotors = 0;
}


MotorController::MotorController(uint8_t numMotors, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf) {
    this->numMotors = numMotors;

    for(int i = 0; i < numMotors; i++) {
        (this->motorList).emplace_back(can_intf, i);

        // Register callbacks for the heartbeat and encoder feedback messages
        (this->motorList[i]).setFeedback();
        (this->motorList[i]).setStatus();
    }
}

void MotorController::addMotor(Motor& motor) {
    this->motorList.push_back(motor);
    this->numMotors++;
}

float MotorController::getMotorBusVoltage(uint8_t motorID) {
    return (this->motorList[motorID]).getBusVoltage();
}

float MotorController::getMotorBusCurrent(uint8_t motorID) {
    return (this->motorList[motorID]).getBusCurrent();
}

void MotorController::clearMotorErrors(uint8_t motorID) {
    (this->motorList[motorID]).clearErrors();
}

void MotorController::setMotorState(uint8_t motorID, enum ODriveAxisState state) {
    (this->motorList[motorID]).setMotorState(state);
}

void MotorController::setMotorPosition(uint8_t motorID, float position, float velocity_feedforward, float torque_feedforward) {
    (this->motorList[motorID]).setPosition(position, velocity_feedforward, torque_feedforward);
}

void MotorController::setMotorVelocity(uint8_t motorID, float velocity, float torque_feedforward) {
    (this->motorList[motorID]).setVelocity(velocity, torque_feedforward);
}

void MotorController::setMotorTorque(uint8_t motorID, float torque) {
    (this->motorList[motorID]).setTorque(torque);
    Serial.print("Setting torque: ");
    Serial.println(torque);
}

float MotorController::getMotorPosition(uint8_t motorID) {
    return (this->motorList[motorID]).getMotorPosition();
}

float MotorController::getMotorVelocity(uint8_t motorID) {
    return (this->motorList[motorID]).getMotorVelocity();
}

uint8_t MotorController::getMotorState(uint8_t motorID) {
    return (this->motorList[motorID]).getMotorState();
}

bool MotorController::checkHeartbeat(uint8_t motorID) {
    return (this->motorList[motorID]).checkHeartbeat();
}

bool MotorController::checkFeedback(uint8_t motorID) {
    return (this->motorList[motorID]).checkFeedback();
}

void MotorController::setupOnReceive(const CanMsg& msg) {
    for(auto motor : this->motorList) {
        onReceive(msg, *(motor.getODrive()));
    }
}

void MotorController::setTorque(std::vector<float> torque) {
    this->torque = torque;

    for(int i = 0; i < this-> numMotors; i++) {
        (this->motorList[i]).setTorque(torque[i]);
    }
}

std::vector<float> MotorController::getTorque() {
    return this->torque;
}