
#include <Arduino.h>
#include "motor_can_handler.hpp"
#include "joint_controller.hpp"



  JointController jc(&motors);

// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html

void setup() {

  // motors.addMotor(mc0); //TODO: make support for constructing Motor_Controller objects in Motor constructor

  Serial.begin(115200);

  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);


  Serial.println("Starting ODriveCAN demo");

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  // NEED TO FIX THIS SO THAT onHeartbeat actually updates the object's heartbeat state
  while (!motors.checkHeartbeat(0)) {
    pumpEvents(can_intf);
  }

  Serial.println("found ODrive");



  Serial.print("DC voltage [V]: ");
  Serial.println(motors.getMotorBusVoltage(0));
  Serial.print("DC current [A]: ");
  Serial.println(motors.getMotorBusCurrent(0));

  Serial.println("Enabling closed loop control...");
  while (motors.getMotorState(0) != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    motors.clearMotorErrors(0);
    delay(1);
    motors.setMotorState(0, ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("ODrive running!");

  motors.setup();
  jc.setup();
}



void loop() {
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
                        // Note that on MCP2515-based platforms, this will delay for a fixed 10ms.
                        //
                        // This has been found to reduce the number of dropped messages, however it can be removed
                        // for applications requiring loop times over 100Hz.

  float SINE_PERIOD = 0.5f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();
  
  float phase = t * (TWO_PI / SINE_PERIOD);

  JointAngle targetAngle = {
    ((float)0.55*sin(phase)) + 5.75F, // position
    ((float)0.55*cos(phase) )* (float)(TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
  };

  std::array<JointAngle, NUM_JOINTS> targetConfiguration = {targetAngle};

  // TODO: Have joint controller class handle this automatically
  // joint0->setTargetAngle(targetAngle);
  jc.setTargetConfiguration(targetConfiguration);
  jc.readAngles();
  jc.runPID();
  
  Serial.printf(">joint_angle:%0.4f\n", jc.getAngles(0).angle);
  Serial.printf(">commanded_angle:%0.4f\n", targetAngle.angle);
  delayMicroseconds(10000);
}