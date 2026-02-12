#pragma once
#include "motor_controller.hpp"
#include "ODriveFlexCAN.hpp"


class Motors {
  public:
    Motors();
    void addMotor(Motor_Controller& motor);
    void setFeedback();
    void setStatus();
    float getMotorBusVoltage(uint8_t motorID);
    float getMotorBusCurrent(uint8_t motorID);
    void clearMotorErrors(uint8_t motorID);
    void setMotorState(uint8_t motorID, enum ODriveAxisState state);
    void setMotorPosition(uint8_t motorID, float position, float velocity_feedforward = 0.0F, float torque_feedforward = 0.0F);
    void setMotorVelocity(uint8_t motorID, float velocity, float torque_feedforward = 0.0F);
    void setMotorTorque(uint8_t motorID, float torque);
    float getMotorPosition(uint8_t motorID);
    float getMotorVelocity(uint8_t motorID);
    uint8_t getMotorState(uint8_t  motorID);
    bool checkHeartbeat(uint8_t motorID);
    bool checkFeedback(uint8_t motorID);
    template <typename T>
    void applyToODrives(T& msg);
  private:
    std::vector<Motor_Controller> motor_list;
    int numMotors;
};