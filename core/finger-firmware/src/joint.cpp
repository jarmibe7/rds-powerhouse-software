#include "joint.hpp"

Joint::Joint(const PIDConstants pidConstants) {
    this->encoder = AS5147(true);
    this->pidConstants = pidConstants;
    // this->encoder = encoder;
}

JointAngle Joint::getJointAngle() {
    return this->encoder.getAngle();
}

void Joint::setTargetAngle(const JointAngle& targetAngle) {
    this->targetAngle = targetAngle;
}

void Joint::takeAngleMeasurement() {
    this->encoder.takeMeasurement();
}

float Joint::runPID() {

    static float errorIntegral = 0.0;

    // Proportional
    float angleError = angleDifference(this->targetAngle.angle, this->encoder.getAngle().angle);

    // Integral
    errorIntegral += angleError;

    // Derivative with velocity feedforward
    float velocityError = this->targetAngle.velocity - this->encoder.getAngle().velocity;

    // Calculates output torque
    float newTorque = this->pidConstants.kP * angleError + this->pidConstants.kI * errorIntegral + this->pidConstants.kD * velocityError;
    return newTorque;
}

void Joint::setup() {
    this->encoder.setup();
}