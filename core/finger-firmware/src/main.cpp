
#include <Arduino.h>
#include "motor_can_handler.hpp"
#include "joint_controller.hpp"





  // TODO: clean up how this is implememnted

  // std::vector<Joint> jointList;

  JointController jc(&motors);

  Joint* joint0;



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
  // while (motors.getMotorState(0) != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
  //   motors.clearMotorErrors(0);
  //   delay(1);
  //   motors.setMotorState(0, ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

  //   // Pump events for 150ms. This delay is needed for two reasons;
  //   // 1. If there is an error condition, such as missing DC power, the ODrive might
  //   //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
  //   //    on the first heartbeat response, so we want to receive at least two
  //   //    heartbeats (100ms default interval).
  //   // 2. If the bus is congested, the setState command won't get through
  //   //    immediately but can be delayed.
  //   for (int i = 0; i < 15; ++i) {
  //     delay(10);
  //     pumpEvents(can_intf);
  //   }
  // }

  Serial.println("ODrive running!");



  // const PIDConstants jPID = {0.1, 0.0, 0.0};
  // jointList.emplace_back(jPID, &j1Encoder);
  // jc.setup();
  joint0 = jc.getJoint(0);
  joint0->setup();
  // jointList[0].setup();
  Serial.println("Setup done");
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

  // float position = (t < 5.0) ? 4.0 : 5.7;

  // motors.setMotorPosition(
  //   0,
  //   sin(phase), // position
  //   cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
  // );

  JointAngle targetAngle = {
    ((float)0.6*sin(phase)) + 4.2, // position
    ((float)0.6*cos(phase) )* (float)(TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
  };

  // JointAngle targetAngle = {
  //   position, // position
  //   0 // velocity feedforward (optional)
  // };

  // TODO: Have joint controller class handle this automatically
  joint0->setTargetAngle(targetAngle);

  // jc.readAngles();
  joint0->takeAngleMeasurement();
  float joint0Torque = joint0->runPID();
  float jacobian = (1 / 30.0) * 0.01;

  Serial.print("MOTOR TORQUE: ");
  Serial.println(joint0Torque * jacobian, 10);
  motors.setMotorTorque(0, joint0Torque * jacobian);
  // motors.setMotorTorque(0, 0.5);
  // jc.setJointTorques();

  // Serial.print("Encoder angle:                       ");
  // Serial.println(j1Encoder.getAngle().angle);
  Serial.println(joint0->getJointAngle().angle);



  // print position and velocity for Serial Plotter
  // if (motors.checkFeedback(0)) {
  //   Serial.print("ODrive 0 Position: ");
  //   Serial.print(motors.getMotorPosition(0));
  //   Serial.print(",");
  //   Serial.print("ODrive 0 Velocity: ");
  //   Serial.println(motors.getMotorVelocity(0));
  // }
  delayMicroseconds(10000);
}