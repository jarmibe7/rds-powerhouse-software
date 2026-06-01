#include "joint.hpp"

#define NUM_JOINTS 1

class JointController {
    public:
        JointController(MotorController& motors);
        // void setJointTorques();
        void readAngles();
        void setup();
        JointAngle getAngles(uint8_t jointID);
        // Joint* getJoint(uint8_t jointID);
        void setTargetConfiguration(const std::array<JointAngle, NUM_JOINTS> newTargetConfiguration);
        void runPID();
        void clearErrors();
    private:
        std::vector<Joint> jointList;
        MotorController& motors; // TODO: make this not use a pointer
        // std::vector<JointAngle> targetConfiguration;
        std::vector<float> jointTorqueList;
        float jacobian;
        std::vector<float> motorTorqueFromJointTorque(std::vector<float>& motorTorques);
};