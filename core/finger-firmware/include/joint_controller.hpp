#include "joint.hpp"

class JointController {
    public:
        JointController(MotorController* motors);
        void setJointTorques();
        void readAngles();
        void setup();
        JointAngle getAngles(uint8_t jointID);
        Joint* getJoint(uint8_t jointID);
    private:
        std::vector<Joint> jointList;
        MotorController* motors; // TODO: make this not use a pointer
};