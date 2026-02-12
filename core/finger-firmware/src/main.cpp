
#include <Arduino.h>
// #include "arduino_freertos.h"
// #include "ODriveCAN.h"
// #include "motor_controller.hpp"
#include "motor_can_handler.hpp"


// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html


/* Configuration of example sketch -------------------------------------------*/

// CAN bus baudrate. Make sure this matches for every device on the bus


// Uncomment below the line that corresponds to your hardware.
// See also "Board-specific settings" to adapt the details for your hardware setup.

#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
// #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
// #define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.


/* Board-specific includes ---------------------------------------------------*/


// struct ODriveStatus; // hack to prevent teensy compile error



/* Example sketch ------------------------------------------------------------*/



// // Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

Motor_Controller mc0(&odrv0, ODRV0_NODE_ID);



// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;





void setup() {

  motors.addMotor(mc0);



  Serial.begin(115200);

  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);


  Serial.println("Starting ODriveCAN demo");

  // Register callbacks for the heartbeat and encoder feedback messages
  // odrv0.onFeedback(onFeedback, &odrv0_user_data);                              BIG CHANGE HERE
  // odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  motors.setFeedback(0, onFeedback);
  motors.setStatus(0, onHeartbeat);


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
}



void loop() {
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
                        // Note that on MCP2515-based platforms, this will delay for a fixed 10ms.
                        //
                        // This has been found to reduce the number of dropped messages, however it can be removed
                        // for applications requiring loop times over 100Hz.

  float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();
  
  float phase = t * (TWO_PI / SINE_PERIOD);

  motors.setMotorPosition(
    0,
    sin(phase), // position
    cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
  );

  // print position and velocity for Serial Plotter
  if (odrv0_user_data.received_feedback) {
    odrv0_user_data.received_feedback = false;
    Serial.print("ODrive 0 Position: ");
    Serial.print(motors.getMotorPosition(0));
    Serial.print(",");
    Serial.print("ODrive 0 Velocity: ");
    Serial.println(motors.getMotorVelocity(0));
  }
}