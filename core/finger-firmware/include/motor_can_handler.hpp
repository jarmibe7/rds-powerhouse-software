#pragma once

#include "motor_controller.hpp"

// Create CANbus object
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// Create motor controller object
MotorController motors(4, can_intf);

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
    motors.parseCanMsg(msg);
}

bool setupCan() {
    Serial.println("initializing CAN");
    can_intf.begin();
    can_intf.setBaudRate(CAN_BAUDRATE);
    can_intf.setMaxMB(16);
    can_intf.enableFIFO();
    can_intf.enableFIFOInterrupt();
    can_intf.onReceive(onCanMessage);
    return true;
}
