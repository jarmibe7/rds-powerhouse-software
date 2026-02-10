#pragma once

// #include "arduino_freertos.h"
#include <list>
#include "ODriveCAN.h"


// #include "ODriveFlexCAN.hpp"



class Motor_Controller {
  public:
    Motor_Controller(ODriveCAN* odrive, int node_id);
  private:
    int nodeId;
    ODriveCAN* odrive;
};


class Motors {
  public:
    Motors();
    void addMotor(Motor_Controller motor);
    // void onCanMessage(const CanMsg& msg);
  private:
    std::list<Motor_Controller> motor_list;
    int numMotors;
};

