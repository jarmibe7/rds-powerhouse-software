#include "joint_controller.hpp"

JointController::JointController(std::vector<Joint>& jointList, MotorController* motors) : jointList(jointList) {
    // this->jointList = jointList;
    this->motors = motors;
}

void JointController::setJointTorques() {
    // TODO: Properly implement this to use jacobian and calculate with multiple motors/tendons
    Serial.println("RUNNING PID");
    float jointTorque = this->jointList[0].runPID();

    // Inverse gear ratio times pulley radius (in m)
    float jacobian = (1 / 30.0) * 0.01;

    motors->setMotorTorque(0, jacobian * jointTorque);
}

void JointController::readAngles() {
    // Serial.println(this->jointList.size());
    for (auto joint : this->jointList) {
        joint.takeAngleMeasurement();
    }
}