/**
 * @brief Red Robot 2026 main code!
 * 
 * Read the comments below to figure out how to get started!
 */

// Libraries with driver code (don't remove these!)
#include <Arduino.h>                // make sure this is first
#include "RedRobot.h"

// Declare some global variables
bool started; // boolean (true or false) to keep track of if we should start
int counter;  // integer to hold a count value

// This function runs ONCE, when the microcontroller boots up
void setup() {
    // Initialize all components on the robot before doing any work
  RedRobot::InitRedRobot();         // call this first
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();            // call this to activate the AutonomousLoop

  // Initialize our variables
  started = false;
  counter = 0;
}

// This function runs over and over (in a loop) when you are in autonomous mode.
// As an example, this code moves out of the starting box at start, and 
// Look at RedRobot.h to see the robot interface: what functions you can call and how.
void AutonomousLoop() {

    if (!started) { // e.g. get out of the starting box
        started = true;

        // move forward - set motors 2 and 3 to speed 30
        RedRobot::WriteDCMotor(2, 30);
        RedRobot::WriteDCMotor(3, 30);

        // wait 1000 milliseconds = 1 second
        delay(1000);

        // turn right - set motor 2 to speed 30 and motor 3 to -30
        RedRobot::WriteDCMotor(2, 30);
        RedRobot::WriteDCMotor(3, -30);

        delay(500); // half a second

        // stop the motors
        RedRobot::WriteDCMotor(2, 0);
        RedRobot::WriteDCMotor(3, 0);
    }
    
    // repeatedly move the servo and get in scoring position
    // remember that the below code will run repeatedly since we are in
    // the AutonomousLoop!

    // set all servos to angle 0
    RedRobot::SetServo(0, 0);
    RedRobot::SetServo(1, 0);
    RedRobot::SetServo(2, 0);
    RedRobot::SetServo(3, 0);

    // move forward at speed 50
    RedRobot::WriteDCMotor(2, 50);
    RedRobot::WriteDCMotor(3, 50);

    delay(1000);

    // stop the motors
    RedRobot::WriteDCMotor(2, 0);
    RedRobot::WriteDCMotor(3, 0);

    // set the motors to different angles
    RedRobot::SetServo(0, 90);
    RedRobot::SetServo(1, 180);
    RedRobot::SetServo(2, 90);
    RedRobot::SetServo(3, 180);

    delay(500);

    // move backward
    RedRobot::WriteDCMotor(2, -50);
    RedRobot::WriteDCMotor(3, -50);

    delay(1000);
    
    // Print to the serial console to debug!
    Serial.print("[AUTO] Running ");
    Serial.print(counter);
    Serial.println();

    // increment the counter
    counter = counter + 1;
}
