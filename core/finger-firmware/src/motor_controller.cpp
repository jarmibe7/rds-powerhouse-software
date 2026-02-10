#include "motor_controller.hpp"

Motor_Controller::Motor_Controller(ODriveCAN* odrive, int node_id) {
    this->nodeId = node_id;
    this->odrive = odrive;
}

Motors::Motors() {
    this->numMotors = 0;
}

void Motors::addMotor(Motor_Controller motor) {
    this->motor_list.push_back(motor);
    this->numMotors++;
}

// void Motors::onCanMessage(const CanMsg& msg) {

// }