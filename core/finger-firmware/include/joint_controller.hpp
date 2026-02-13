#include "joint.hpp"

class JointController {
    public:
        JointController(std::vector<Joint>& jointList, Motors* motors);
        void setJointTorques();
        void readAngles();
    private:
        std::vector<Joint> jointList;
        Motors* motors; // TODO: make this not use a pointer
};