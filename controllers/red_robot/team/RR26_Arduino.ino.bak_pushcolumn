/**
 * @brief Red Robot 2026 main code!
 *
 * AUTONOMOUS: "push the column" (red alliance, robot starts in the red start
 * zone facing the pipe factories)
 *   1. drive out of the start zone, using the black tape at the bottom of the
 *      zone as a position check
 *   2. turn left and drive over until the robot is underneath the column of
 *      three pipes on our side (the 3rd pipe counting down)
 *   3. turn left to face the smokestacks, close the claw paddles around that
 *      pipe and push it up the column - it shoves pipes 2 and 1 ahead of it
 *      until all three are in the stacking area
 *
 * The RedRobot API has no wheel encoders, so every move is TIMED. The numbers
 * in the TUNING block were measured in the Webots sim with fresh batteries;
 * re-time them on the real robot (time a 1 m drive and a 360 deg spin).
 */

// Libraries with driver code (don't remove these!)
#include <Arduino.h>                // make sure this is first
#include "RedRobot.h"

// ------------------------------------------------------------------ TUNING
const int LEFT = 2, RIGHT = 3;            // drive motor numbers

const int DRIVE_POWER = 40;               // ~0.34 m/s in the sim
const int PUSH_POWER = 25;                // ~0.21 m/s - slow so the pipes stay upright
const int TURN_POWER = 30;                // spin in place, ~170 deg/s in the sim

const unsigned long FIND_TAPE_TIMEOUT_MS = 1500;  // give up looking for the tape after this
const unsigned long PAST_TAPE_MS = 1000;   // tape -> turning point (~0.37 m)
const unsigned long TURN1_MS = 538;        // 1st left turn (90 deg) at TURN_POWER
const unsigned long TURN2_MS = 555;        // 2nd left turn (90 deg) - the heading here decides
                                           // whether the pushed pipes stay in a straight line
const unsigned long ACROSS_MS = 930;       // drive across to the pipe column (~0.32 m)
const unsigned long REACH_PIPE_MS = 500;   // creep until pipe 3 is between the paddles
const unsigned long PUSH_MS = 4800;        // push the column into the stacking area

const uint16_t TAPE_THRESHOLD = 1200;      // line sensor: above = black tape

// claw paddles (servo 0 = left paddle, servo 1 = right paddle)
const int PADDLE_OPEN_L = 0, PADDLE_OPEN_R = 0;        // straight ahead
const int PADDLE_CLOSED_L = -60, PADDLE_CLOSED_R = 60; // ~30 deg inward: cradles one pipe

// ------------------------------------------------------------------ helpers
bool autoDone = false;

void drive(int left, int right) {
  RedRobot::WriteDCMotor(LEFT, left);
  RedRobot::WriteDCMotor(RIGHT, right);
}

void stopAndSettle(unsigned long ms = 200) {
  drive(0, 0);
  delay(ms);
}

void paddles(int left, int right) {
  RedRobot::SetServo(0, left);
  RedRobot::SetServo(1, right);
}

bool seesTape() {
  uint16_t v[6];
  RedRobot::ReadLineSensor(v);
  for (int i = 0; i < 6; i++)
    if (v[i] > TAPE_THRESHOLD) return true;
  return false;
}

void turnLeft(unsigned long ms) {
  drive(-TURN_POWER, TURN_POWER);
  delay(ms);
  stopAndSettle();
}

// This function runs ONCE, when the microcontroller boots up
void setup() {
  // Initialize all components on the robot before doing any work
  RedRobot::InitRedRobot();         // call this first
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();            // call this to activate the AutonomousLoop
}

// This function runs over and over (in a loop) when you are in autonomous mode.
// The routine runs once; after that the robot waits for teleop (press X).
void AutonomousLoop() {
  if (autoDone) return;   // motors are already stopped; don't keep writing to them,
                          // or controller input before X gets wiped out every loop
  autoDone = true;
  Serial.println("[AUTO] push-the-column routine starting");

  paddles(PADDLE_OPEN_L, PADDLE_OPEN_R);

  // 1. out of the start zone until the line sensors hit the tape at its bottom edge
  drive(DRIVE_POWER, DRIVE_POWER);
  unsigned long t0 = millis();
  while (!seesTape() && millis() - t0 < FIND_TAPE_TIMEOUT_MS) {
  }
  Serial.printf("[AUTO] tape %s after %lu ms\n", seesTape() ? "found" : "NOT found", millis() - t0);
  delay(PAST_TAPE_MS);              // keep going to the turning point
  stopAndSettle();

  // 2. turn left (now facing the red trench side) and drive under the pipe column
  turnLeft(TURN1_MS);
  drive(DRIVE_POWER, DRIVE_POWER);
  delay(ACROSS_MS);
  stopAndSettle();

  // 3. turn left to face the smokestacks, catch pipe 3 between the paddles
  turnLeft(TURN2_MS);
  drive(PUSH_POWER, PUSH_POWER);
  delay(REACH_PIPE_MS);
  paddles(PADDLE_CLOSED_L, PADDLE_CLOSED_R);

  // ...and push the whole column into the stacking area
  delay(PUSH_MS);
  stopAndSettle();

  // let go and back off a little so we're not touching the stack
  paddles(PADDLE_OPEN_L, PADDLE_OPEN_R);
  delay(300);
  drive(-PUSH_POWER, -PUSH_POWER);
  delay(400);
  stopAndSettle();
  Serial.println("[AUTO] done - press X for teleop");
}
