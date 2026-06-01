#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
// #include "ODriveFlexCAN.hpp"
// #include "motor_controller.hpp"

#define CAN_BAUDRATE 1000000

void sendCanMsgInt(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf, uint32_t id, bool extended, uint8_t length, int32_t data);

void sendCanBuffer(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf, uint32_t id, bool extended, uint8_t length, uint8_t* buffer);
