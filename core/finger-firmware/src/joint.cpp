#include "joint.hpp"

Joint::Joint(const PIDConstants pidConstants) {
    this->encoder = AS5147(false);
    this->pidConstants = pidConstants;
    // this->encoder = encoder;
}

JointAngle Joint::getJointAngle() {
    return this->encoder.getAngle();
}

void Joint::setTargetAngle(JointAngle targetAngle) {
    this->targetAngle = targetAngle;
}

void Joint::takeAngleMeasurement() {
    Serial.println("Taking joint angle measurement");
    this->encoder.takeMeasurement();
}

float Joint::runPID() {

    static float errorIntegral = 0.0;

    // Proportional
    float angleError = angleDifference(this->targetAngle.angle, this->encoder.getAngle().angle);
    Serial.print("JOINT ANGLE: ");
    Serial.println(this->encoder.getAngle().angle);
    Serial.print("TARGET ANGLE: ");
    Serial.println(this->targetAngle.angle);
    

    // Integral
    errorIntegral += angleError;

    // Derivative with velocity feedforward
    float velocityError = this->targetAngle.velocity - this->encoder.getAngle().velocity;

    // Calculates output torque
    return this->pidConstants.kP * angleError + this->pidConstants.kI * errorIntegral + this->pidConstants.kD * velocityError;
}

void Joint::setup() {
    this->encoder.setup();
}