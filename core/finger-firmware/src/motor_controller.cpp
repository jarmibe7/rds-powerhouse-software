// #include "motor_controller.hpp"

// bool setupCan() {
//   can_intf.begin();
//   can_intf.setBaudRate(CAN_BAUDRATE);
//   can_intf.setMaxMB(16);
//   can_intf.enableFIFO();
//   can_intf.enableFIFOInterrupt();
//   can_intf.onReceive(onCanMessage);
//   return true;
// }


// // Called every time a Heartbeat message arrives from the ODrive
// void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
//   ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
//   odrv_user_data->last_heartbeat = msg;
//   odrv_user_data->received_heartbeat = true;
// }

// // Called every time a feedback message arrives from the ODrive
// void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
//   ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
//   odrv_user_data->last_feedback = msg;
//   odrv_user_data->received_feedback = true;
// }