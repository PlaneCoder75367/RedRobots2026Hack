/**
 * Example autonomous for Red Robot 2026: drive out of the start zone, find the
 * black tape, follow it to its end and stop. Uses only the RedRobot.h API, so
 * the same file runs on the real robot and in the Webots simulator.
 *
 * Try it in the sim: copy this file over controllers/red_robot/team/RR26_Arduino.ino
 * (keep a backup of yours), then reload the world.
 *
 * THRESHOLD and the gains are starting points - tune them on the real field:
 * print the raw ReadLineSensor values over the floor and over the tape first.
 */
#include <Arduino.h>
#include "RedRobot.h"

// ---- tuning -------------------------------------------------------------
const int LEFT_MOTOR = 2;
const int RIGHT_MOTOR = 3;
const uint16_t THRESHOLD = 1200;   // above = black tape (white floor ~150-300, tape ~2000+)
const int SEARCH_SPEED = 30;       // while looking for the line
const int BASE_SPEED = 35;         // while following
const float KP = 9.0;              // steering per unit of line-position error
const float KD = 25.0;
const unsigned long LOST_MS = 250; // line gone this long = end of the line

// ---- state ----------------------------------------------------------------
enum State { SEARCH, FOLLOW, DONE };
State state = SEARCH;
float lastError = 0;
unsigned long lastSeen = 0;

void setup() {
  RedRobot::InitRedRobot();
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();
}

void drive(int left, int right) {
  RedRobot::WriteDCMotor(LEFT_MOTOR, constrain(left, -100, 100));
  RedRobot::WriteDCMotor(RIGHT_MOTOR, constrain(right, -100, 100));
}

// Returns true if any sensor sees tape; position is -2.5 (line under lf0, the
// robot's right side) .. +2.5 (under lf5, the left side).
bool linePosition(float &position) {
  uint16_t v[6];
  RedRobot::ReadLineSensor(v);
  float sum = 0, weighted = 0;
  for (int i = 0; i < 6; i++) {
    float w = v[i] > THRESHOLD ? (float)(v[i] - THRESHOLD) : 0.0f;
    sum += w;
    weighted += w * (i - 2.5f);
  }
  if (sum <= 0) return false;
  position = weighted / sum;
  return true;
}

void AutonomousLoop() {
  float pos = 0;
  bool seen = linePosition(pos);

  switch (state) {
    case SEARCH:
      drive(SEARCH_SPEED, SEARCH_SPEED);
      if (seen) {
        Serial.println("[AUTO] found the line");
        state = FOLLOW;
        lastSeen = millis();
      }
      break;

    case FOLLOW:
      if (seen) {
        lastSeen = millis();
        float error = pos;                       // + = line is to the LEFT
        float turn = KP * error + KD * (error - lastError);
        lastError = error;
        drive(BASE_SPEED - (int)turn, BASE_SPEED + (int)turn);
      } else if (millis() - lastSeen > LOST_MS) {
        Serial.println("[AUTO] end of the line - stopping");
        drive(0, 0);
        state = DONE;
      }
      break;

    case DONE:
      drive(0, 0);
      break;
  }
}
