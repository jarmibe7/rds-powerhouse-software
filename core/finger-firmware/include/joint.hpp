#include  <functional>
#include "motor_controller.hpp"
#include "encoder.hpp"




struct PIDConstants {
    float kP;
    float kI;
    float kD;
};

class Joint {
    public:
        Joint(const PIDConstants pidConstants);
        JointAngle getJointAngle();
        void setTargetAngle(const JointAngle& targetAngle);
        void takeAngleMeasurement();
        float runPID();
        void setup();
    private:
        AS5147 encoder;
        JointAngle targetAngle;
        PIDConstants pidConstants;
};