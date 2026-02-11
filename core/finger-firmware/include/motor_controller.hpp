#pragma once

// #include "arduino_freertos.h"
#include <functional>
#include <vector>
#include "ODriveCAN.h"
#include <Arduino.h>  


// #include "ODriveFlexCAN.hpp"



class Motor_Controller {
  public:
    Motor_Controller(ODriveCAN* odrive, int node_id);
    void setFeedback(void (*callback)(Get_Encoder_Estimates_msg_t& feedback, void* user_data), void* user_data);
    void setStatus(void (*callback)(Heartbeat_msg_t& feedback, void* user_data), void* user_data = nullptr);
    float getBusVoltage();
    float getBusCurrent();
    void clearErrors();
    void setMotorState(enum ODriveAxisState state);
    void setPosition(float position, float velocity_feedforward = 0.0F, float torque_feedforward = 0.0F);
    void setVelocity(float velocity, float torque_feedforward = 0.0F);
    void setTorque(float torque);
  private:
    int nodeId;
    ODriveCAN* odrive;
    Get_Bus_Voltage_Current_msg_t vbus;
};


class Motors {
  public:
    Motors();
    void addMotor(Motor_Controller& motor);
    void setFeedback(uint8_t motorID, void (*callback)(Get_Encoder_Estimates_msg_t& feedback, void* user_data), void* user_data);
    void setStatus(uint8_t motorID, void (*callback)(Heartbeat_msg_t& feedback, void* user_data), void* user_data = nullptr);
    float getMotorBusVoltage(uint8_t motorID);
    float getMotorBusCurrent(uint8_t motorID);
    void clearMotorErrors(uint8_t motorID);
    void setMotorState(uint8_t motorID, enum ODriveAxisState state);
    void setMotorPosition(uint8_t motorID, float position, float velocity_feedforward = 0.0F, float torque_feedforward = 0.0F);
    void setMotorVelocity(uint8_t motorID, float velocity, float torque_feedforward = 0.0F);
    void setMotorTorque(uint8_t motorID, float torque);
  private:
    std::vector<Motor_Controller> motor_list;
    int numMotors;
};

