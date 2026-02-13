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
        Joint(const PIDConstants pidConstants);
        void setupAngleMeasurement(std::function<float(void)> measureAngle);
        JointAngle getJointAngle();
        void setTargetAngle(JointAngle targetAngle);
        void takeAngleMeasurement();
        float runPID();
    private:
        std::function<float(void)> measureAngle;
        JointAngle measuredAngle;
        JointAngle targetAngle;
        PIDConstants pidConstants;
};