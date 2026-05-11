#pragma once

// #include "arduino_freertos.h"
#include <functional>
#include <vector>
#include "ODriveCAN.h"
#include <Arduino.h>  
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
#include "can_handler.hpp"


#define GEAR_RATIO 10.0f
#define POLE_PAIRS 14.0f
#define MOTOR_KT 0.056f


struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

class Motor {
  public:
    Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int motor_id);
    void setup();
    void setTorque(float torque);
    float getMotorPosition();
    float getMotorVelocity();
    float getMotorCurrent();
    float getMotorTemperature();
    uint8_t getMotorError();
    void updateFeedback(float position, float velocity, float current, float temperature, uint8_t error);

  private:
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf;
    int motorId;
    float position;
    float velocity;
    float current;
    float temperature;
    uint8_t error;
};

void pack_cmd(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, uint8_t motorID, float p_des, float v_des, float kp, float kd, float t_ff);
void enterMotorControlMode(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, uint8_t motorID);

int float_to_uint(float x, float x_min, float x_max, unsigned int bits);