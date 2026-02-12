#pragma once

// #include "arduino_freertos.h"
#include <functional>
#include <vector>
#include "ODriveCAN.h"
#include <Arduino.h>  
#include <FlexCAN_T4.h>


// #include "ODriveFlexCAN.hpp"

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};


void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

class Motor_Controller {
  public:
    Motor_Controller(ODriveCAN* odrive, int node_id);
    void setFeedback();
    void setStatus();
    float getBusVoltage();
    float getBusCurrent();
    void clearErrors();
    void setMotorState(enum ODriveAxisState state);
    void setPosition(float position, float velocity_feedforward = 0.0F, float torque_feedforward = 0.0F);
    void setVelocity(float velocity, float torque_feedforward = 0.0F);
    void setTorque(float torque);
    float getMotorPosition();
    float getMotorVelocity();
    uint8_t getMotorState();
    bool checkHeartbeat();
    bool checkFeedback();
    ODriveCAN* getODrive();
  private:
    int nodeId;
    ODriveCAN* odrive;
    Get_Bus_Voltage_Current_msg_t vbus;
    ODriveUserData user_data;
};


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

