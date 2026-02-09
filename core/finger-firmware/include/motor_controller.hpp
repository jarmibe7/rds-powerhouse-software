// #pragma once

// #include "ODriveCAN.h"
// #include "ODriveFlexCAN.hpp"


// // CAN bus baudrate. Make sure this matches for every device on the bus
// #define CAN_BAUDRATE 250000

// // ODrive node_id for odrv0
// #define ODRV0_NODE_ID 0

// extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// struct ODriveStatus;

// struct ODriveUserData {
//   Heartbeat_msg_t last_heartbeat;
//   bool received_heartbeat;
//   Get_Encoder_Estimates_msg_t last_feedback;
//   bool received_feedback;
// };

// // Called every time a Heartbeat message arrives from the ODrive
// void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);

// // Called every time a feedback message arrives from the ODrive
// void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

// // Called for every message that arrives on the CAN bus
// void onCanMessage(const CanMsg& msg);

// bool setupCan(void);

// // // Keep some application-specific user data for every ODrive.
// // extern ODriveUserData odrv0_user_data;
