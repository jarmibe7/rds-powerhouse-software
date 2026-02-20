#include "joint_controller.hpp"

JointController::JointController(MotorController& motors) : motors(motors) , jointTorqueList(NUM_JOINTS) {
    // this->jointList = jointList;
    const PIDConstants jPID = {1200.0, 0.0, 0.0};

    // this->jointList
    this->jointList = {Joint(jPID)};
    // this->motors = motors;
    this->jacobian = (1 / 30.0) * 0.01;
}

// void JointController::setJointTorques() {
//     // TODO: Properly implement this to use jacobian and calculate with multiple motors/tendons
//     Serial.println("RUNNING PID");
//     float jointTorque = this->jointList[0].runPID();

//     // Inverse gear ratio times pulley radius (in m)
//     float jacobian = (1 / 30.0) * 0.01;

//     motors->setMotorTorque(0, jacobian * jointTorque);
// }

void JointController::readAngles() {
    // Serial.println(this->jointList.size());
    for (auto& joint : this->jointList) {
        joint.takeAngleMeasurement();
    }
}

void JointController::setup() {
    for (auto& joint : this->jointList) {
        joint.setup();
    }
}

JointAngle JointController::getAngles(uint8_t jointID) {
    return jointList[jointID].getJointAngle();
}

void JointController::setTargetConfiguration(const std::array<JointAngle, NUM_JOINTS> newTargetConfiguration) {
    for(int i = 0; i < NUM_JOINTS; i++) {
        this->jointList[i].setTargetAngle(newTargetConfiguration[i]);
    }
}

void JointController::runPID() {
    uint8_t i = 0;
    for (auto& joint : this->jointList) {
        this->jointTorqueList[i] = joint.runPID();

        i++;
    }

    std::vector<float> motorTorques = {};

    //TODO: FIX THIS TO USE JACOBIAN PROPERLY
    this->motors.setTorque(motorTorqueFromJointTorque(motorTorques));
}

std::vector<float> JointController::motorTorqueFromJointTorque(std::vector<float>& motorTorques) {
    for(int i = 0; i < NUM_JOINTS; i++) {
        motorTorques.push_back(this->jointTorqueList[i] * jacobian); // Fix this
    }
    return motorTorques;
}