#include "joint.hpp"

Joint::Joint(const PIDConstants pidConstants) {
    this->pidConstants = pidConstants;
}

void Joint::setupAngleMeasurement(std::function<float(void)> measureAngle) {
    this->measureAngle = measureAngle;
}

JointAngle Joint::getJointAngle() {
    return this->measuredAngle;
}

void Joint::setTargetAngle(JointAngle targetAngle) {
    this->targetAngle = targetAngle;
}

void Joint::takeAngleMeasurement() {
    this->measuredAngle.angle = this->measureAngle();
    // TODO: Implement velocity measurement
}

float Joint::runPID() {

    static float errorIntegral = 0.0;

    // Proportional
    float angleError = this->targetAngle.angle - this->measuredAngle.angle;

    // Integral
    errorIntegral += angleError;

    // Derivative with velocity feedforward
    float velocityError = this->targetAngle.velocity - this->measuredAngle.velocity;

    // Calculates output torque
    return this->pidConstants.kP * angleError + this->pidConstants.kI * errorIntegral + this->pidConstants.kD * velocityError;
}

