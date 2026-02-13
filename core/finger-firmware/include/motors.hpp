#pragma once
#include "motor_controller.hpp"


class Motors {
  public:
    Motors();
    Motors(uint8_t numMotors, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf);
    void addMotor(Motor_Controller& motor);
    float getMotorBusVoltage(uint8_t motorID);
    float getMotorBusCurrent(uint8_t motorID);
    void clearMotorErrors(uint8_t motorID);
    void setMotorState(uint8_t motorID, enum ODriveAxisState state);
    void setMotorPosition(uint8_t motorID, float position, float velocity_feedforward = 0.0, float torque_feedforward = 0.0);
    void setMotorVelocity(uint8_t motorID, float velocity, float torque_feedforward = 0.0);
    void setMotorTorque(uint8_t motorID, float torque);
    float getMotorPosition(uint8_t motorID);
    float getMotorVelocity(uint8_t motorID);
    uint8_t getMotorState(uint8_t  motorID);
    bool checkHeartbeat(uint8_t motorID);
    bool checkFeedback(uint8_t motorID);
    void setupOnReceive(const CanMsg& msg);
    void setTorque(std::vector<float> torque);
    std::vector<float> getTorque();
  private:
    std::vector<Motor_Controller> motor_list;
    int numMotors;
    std::vector<float> torque;
};