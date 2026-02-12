#pragma once

#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
// #include "ODriveFlexCAN.hpp"
#include "motors.hpp"

#define CAN_BAUDRATE 250000

// ODrive node_id for odrv0
#define ODRV0_NODE_ID 0

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

Motors motors(1, can_intf);

// Instantiate ODrive objects
// ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
// ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

// Motor_Controller mc0(&odrv0, ODRV0_NODE_ID);




// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
    motors.setupOnReceive(msg);
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
