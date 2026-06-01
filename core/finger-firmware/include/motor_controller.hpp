#pragma once
#include "motor.hpp"

enum class MotorID {
    Motor0,
    Motor1,
    Motor2,
    Motor3
};

class MotorController {
  public:
    MotorController();
    MotorController(uint8_t numMotors, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf);
    void addMotor(Motor& motor);
    float getMotorBusVoltage(uint8_t motorID);
    void clearMotorErrors(uint8_t motorID);
    void setMotorTorque(uint8_t motorID, float torque);
    float getMotorPosition(uint8_t motorID);
    float getMotorVelocity(uint8_t motorID);
    uint8_t getMotorState(uint8_t  motorID);
    void setTorque(std::vector<float> torque);
    std::vector<float> getTorque();
    void setup();
    void parseCanMsg(const CanMsg& msg);
  private:
    std::vector<Motor> motorList;
    int numMotors;
    std::vector<float> torque;
};