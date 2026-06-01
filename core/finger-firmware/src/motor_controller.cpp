#include "motor_controller.hpp"

MotorController::MotorController() {
    this->numMotors = 0;
}


MotorController::MotorController(uint8_t numMotors, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf) {
    this->numMotors = numMotors;

    for(uint8_t i = 0; i < numMotors; i++) {
        (this->motorList).emplace_back(can_intf, i);
    }
}

void MotorController::addMotor(Motor& motor) {
    this->motorList.push_back(motor);
    this->numMotors++;
}

void MotorController::setMotorTorque(uint8_t motorID, float torque) {
    (this->motorList[motorID]).setTorque(torque);
}

float MotorController::getMotorPosition(uint8_t motorID) {
    return (this->motorList[motorID]).getMotorPosition();
}

float MotorController::getMotorVelocity(uint8_t motorID) {
    return (this->motorList[motorID]).getMotorVelocity();
}

void MotorController::setTorque(std::vector<float> torque) {
    this->torque = torque;

    for(int i = 0; i < this->numMotors; i++) {
        (this->motorList[i]).setTorque(torque[i]);
    }
}

std::vector<float> MotorController::getTorque() {
    return this->torque;
}

void MotorController::setup() {
    for(int i = 0; i < this->numMotors; i++) {
        (this->motorList[i]).setup();
    }
}

void MotorController::parseCanMsg(const CanMsg& msg) {

    // Check if message is motor feedback message 
    if(msg.flags.extended && msg.id >> 8 == 0x29) {
        uint8_t motorID = msg.id && 0xFF;

        float motorPosition = 0.1 * (float)(msg.buf[0] << 8 | (msg.buf[1]));
        float motorVelocity = (10.0 /(GEAR_RATIO * POLE_PAIRS)) * (float)(msg.buf[2] << 8 | (msg.buf[3]));
        float motorCurrent = 0.01 * (float)(msg.buf[4] << 8 | (msg.buf[5]));
        float motorTemperature = 0.01 * (float)(msg.buf[4] << 8 | (msg.buf[5]));
        uint8_t motorError = msg.buf[7];

        this->motorList[motorID].updateFeedback(motorPosition, motorVelocity, motorCurrent, motorTemperature, motorError);
    }
}