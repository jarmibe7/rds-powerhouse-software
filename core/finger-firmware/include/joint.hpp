#include  <functional>
#include "motors.hpp"
#include "encoder.hpp"




struct PIDConstants {
    float kP;
    float kI;
    float kD;
};

class Joint {
    public:
        Joint(const PIDConstants pidConstants, Encoder& encoder);
        JointAngle getJointAngle();
        void setTargetAngle(JointAngle targetAngle);
        void takeAngleMeasurement();
        float runPID();
        void setup();
    private:
        Encoder encoder;
        JointAngle targetAngle;
        PIDConstants pidConstants;
};