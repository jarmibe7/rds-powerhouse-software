#include "motor_controller.hpp"


// Motor_Controller::Motor_Controller(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int node_id) : odrive(wrap_can_intf(can_intf), node_id) {
    
//     // ODriveCAN odrive(wrap_can_intf(can_intf), node_id); // Standard CAN message ID
//     this->nodeId = node_id;
// }

Motor_Controller::Motor_Controller(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int node_id) : can_intf(can_intf), nodeId(node_id) {}

void Motor_Controller::setFeedback() {
    this->odrive.onFeedback(onFeedback, &(this->user_data));
}

void Motor_Controller::setStatus() {
    this->odrive.onStatus(onHeartbeat, &(this->user_data));
}

float Motor_Controller::getBusVoltage() {
    Serial.println("starting voltage read");
    if (!((this->odrive).request(this->vbus, 1000))) {
        Serial.println("vbus request failed!");
    }
    return this->vbus.Bus_Voltage;
}
float Motor_Controller::getBusCurrent() {
    if (!((this->odrive).request(this->vbus, 1000))) {
        Serial.println("vbus request failed!");
    }
    return this->vbus.Bus_Voltage;
}

void Motor_Controller::clearErrors() {
    this->odrive.clearErrors();
}

void Motor_Controller::setMotorState(enum ODriveAxisState state) {
    this->odrive.setState(state);
}

void Motor_Controller::setPosition(float position, float velocity_feedforward, float torque_feedforward) {
    this->odrive.setPosition(position, velocity_feedforward, torque_feedforward);
}

void Motor_Controller::setVelocity(float velocity, float torque_feedforward) {
    this->odrive.setVelocity(velocity, torque_feedforward);
}

void Motor_Controller::setTorque(float torque) {
    this->odrive.setTorque(torque);
}

float Motor_Controller::getMotorPosition() {
    return this->user_data.last_feedback.Pos_Estimate;
}

float Motor_Controller::getMotorVelocity() {
    return this->user_data.last_feedback.Pos_Estimate;
}

uint8_t Motor_Controller::getMotorState() {
    return this->user_data.last_heartbeat.Axis_State;
}

bool Motor_Controller::checkHeartbeat() {
    bool heartbeat = this->user_data.received_heartbeat;
    this->user_data.received_heartbeat = false;
    return heartbeat;
}

bool Motor_Controller::checkFeedback() {
    bool feedback = this->user_data.received_feedback;
    this->user_data.received_feedback = false;
    return feedback;
}

ODriveCAN* Motor_Controller::getODrive() {
    return &(this->odrive);
}

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}
