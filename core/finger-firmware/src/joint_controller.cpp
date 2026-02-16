#include "joint_controller.hpp"

JointController::JointController(MotorController* motors) {
    // this->jointList = jointList;
    const PIDConstants jPID = {1000.0, 0.0, 0.0};
    this->jointList = {Joint(jPID)};
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

void JointController::setup() {
    for (auto joint : this->jointList) {
        joint.setup();
    }
}

JointAngle JointController::getAngles(uint8_t jointID) {
    return jointList[jointID].getJointAngle();
}

Joint* JointController::getJoint(uint8_t jointID) {
    return &jointList[jointID];
}