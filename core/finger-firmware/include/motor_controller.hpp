#pragma once

// #include "arduino_freertos.h"
#include <functional>
#include <vector>
#include "ODriveCAN.h"
#include <Arduino.h>  
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"


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
    Motor_Controller(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int node_id);
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
    ODriveCAN odrive;
    Get_Bus_Voltage_Current_msg_t vbus;
    ODriveUserData user_data;
};

