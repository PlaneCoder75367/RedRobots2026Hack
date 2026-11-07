/**
 * Red Robot 2026 - RED auto "trench + stack"  (Ronav's line follower, fixed)
 *
 * START POSE: inside the red start zone, nose at the start of the STRAIGHT black
 * line (the one that runs from the start zone to the red trench), robot aimed
 * along that line, paddles closed. In the sim this is RED1 in field_config.json.
 *
 * ROUTE (20 s auto, ~10 s used):
 *   1. CALIBRATE  read the floor under the sensors (Ronav's CalibrateLine)
 *   2. FIND_LINE  creep forward until the tape shows up
 *   3. FOLLOW     P-D line follow along the straight line. The middle pipe of our
 *                 column sits ON this line, so the robot scoops it up between the
 *                 paddles and pushes it all the way to the trench.
 *   4. CURB_PUSH  line ends at the trench curb: keep pushing briefly so the pipe
 *                 tips over the curb and lies flat in the trench.
 *                 -> exit start zone +20, line to trench +20, trench pipe 10 + auto
 *                    bonus 20, and the trench half of the auto RP.
 *   5. BACKUP     reverse straight back along the line to the pipe column
 *   6. TURN       spin left to face the smokestacks
 *   7. PUSH_STACK push the top column pipe (upright) into a smokestack recess
 *                 -> stack pipe 20 + auto bonus 20
 *   8. RELEASE / BACK_OFF, then wait for teleop (press X)
 *
 * Steps 3-4 are sensor-steered. Steps 5-7 are TIMED (no encoders/gyro yet), so
 * re-time them on the real robot (audit section E: 1 m straight + 360 spin).
 *
 * Fixes vs Ronav's original (see 05_ronav_line_follow_review.md):
 *   #1 steering sign now depends on SENSOR0_ON_RIGHT (true in the sim; CHECK on robot)
 *   #2 drive() defined once          #3 ultrasonic call removed (no such sensor)
 *   #4 autoGo is set in setup()      #5 no isnan() needed any more
 *   #6 uses the straight line to the TRENCH (the curve now ends in the safe zone)
 *   #7 follow timeout raised and the follow ends on "line lost" or the curb
 */
#include <Arduino.h>
#include "RedRobot.h"

// =============================================================== TUNING
const int LEFT_MOTOR = 2, RIGHT_MOTOR = 3;

// ---- line sensor (CHECK ON THE REAL ROBOT: audit section E rows 6-7) ----
// Print ReadLineSensor with the tape under the robot's RIGHT edge. If v[0] is the
// big number, SENSOR0_ON_RIGHT = true. Wrong value = robot steers AWAY from the line.
const bool SENSOR0_ON_RIGHT = true;
// Tape reads HIGHER than floor (sim: floor ~150, tape ~2500; training-slide data:
// floor ~50, tape ~1000). ~10000 = "saturated", the sensor sees nothing (lifted).
const bool LINE_READS_HIGH = true;
const float CENTER = 2.5f;          // middle of sensors 0..5
const float NOISE = 150;            // rises above the floor smaller than this are ignored
const float MIN_SUM = 400;          // total signal below this = no line under us
const uint16_t SATURATED = 9000;    // every sensor above this = lifted / tipped, ignore

// ---- line PID (Ronav's PID struct; D term uses the slides' low-pass filter) ----
const float KP = 10.0f, KI = 0.0f, KD = 1.0f, I_MAX = 2.0f;
const float D_FILTER_A = 0.6f;      // filtered = A*filtered + (1-A)*new  (0 = no filter)
const int FOLLOW_SPEED = 35;        // base speed while following / pushing the pipe
const int FIND_SPEED = 30;          // creeping onto the line

// ---- timing of the sensor-steered part (safety caps, not the plan) ----
const unsigned long CALIBRATE_SETTLE_MS = 100;
const unsigned long FIND_LINE_TIMEOUT_MS = 1500;
const unsigned long FOLLOW_MIN_MS = 800;     // ignore "line lost" this early
const unsigned long FOLLOW_MAX_MS = 4500;    // give up following after this
const unsigned long LOST_MS = 200;           // line gone this long = end of the line
const unsigned long CURB_PUSH_MS = 700;      // push on after the line ends (tips the pipe in)
const int CURB_PUSH_SPEED = 30;

// ---- timed stack leg (sim values; RE-TIME on the real robot) ----
const int DRIVE_POWER = 40;                  // ~0.34 m/s in the sim
const int TURN_POWER = 30;                   // ~170 deg/s in the sim
const int PUSH_POWER = 30;                   // pushing the upright pipe
const unsigned long BACKUP_MS = 900;         // reverse from the trench to the column
const unsigned long TURN_TO_STACK_MS = 600;  // ~98 deg left (line is tilted 8 deg)
const unsigned long STACK_PUSH_MS = 2600;    // drive up, pipe into the smokestack recess
const unsigned long RELEASE_MS = 300;
const unsigned long BACK_OFF_MS = 400;

// ---- claw paddles (servo 0 = left, servo 1 = right; 0 = straight ahead) ----
const int PADDLE_OPEN_L = 0, PADDLE_OPEN_R = 0;
const int PADDLE_CLOSED_L = -60, PADDLE_CLOSED_R = 60;   // ~30 deg inward: cradles a pipe

// =============================================================== helpers
void drive(int left, int right) {              // the ONE drive function (fix #2)
  RedRobot::WriteDCMotor(LEFT_MOTOR, constrain(left, -100, 100));
  RedRobot::WriteDCMotor(RIGHT_MOTOR, constrain(right, -100, 100));
}

void paddles(int left, int right) {
  RedRobot::SetServo(0, left);
  RedRobot::SetServo(1, right);
}

// ---- Ronav's line code ----
uint16_t base[6];

void CalibrateLine() {                         // average 10 reads of plain floor
  uint32_t sum[6] = {0};
  uint16_t v[6];
  for (int n = 0; n < 10; n++) {
    RedRobot::ReadLineSensor(v);
    for (int i = 0; i < 6; i++) sum[i] += v[i];
  }
  for (int i = 0; i < 6; i++) base[i] = sum[i] / 10;
  Serial.printf("[AUTO] floor baseline: %u %u %u %u %u %u\n", base[0], base[1], base[2], base[3], base[4], base[5]);
}

// Weighted average of the sensor indices (training slides), measured above the
// floor baseline. pos = 0..5 in sensor-index order. false = no line / lifted.
bool LinePosition(float &pos) {
  uint16_t v[6];
  RedRobot::ReadLineSensor(v);
  int saturated = 0;
  float sum = 0, wsum = 0;
  for (int i = 0; i < 6; i++) {
    if (v[i] >= SATURATED) saturated++;
    float d = LINE_READS_HIGH ? (float)v[i] - base[i] : (float)base[i] - v[i];
    if (d < NOISE) d = 0;
    sum += d;
    wsum += d * i;
  }
  if (saturated == 6 || sum < MIN_SUM) return false;
  pos = wsum / sum;
  return true;
}

struct PID {                                    // Ronav's PID, D term low-pass filtered
  float kp, ki, kd, iMax;
  float integral = 0, prevErr = 0, dFilt = 0;
  bool first = true;
  void reset() { integral = 0; dFilt = 0; first = true; }
  float update(float err, float dt) {
    integral = constrain(integral + err * dt, -iMax, iMax);
    float deriv = first ? 0 : (err - prevErr) / dt;
    dFilt = D_FILTER_A * dFilt + (1 - D_FILTER_A) * deriv;
    first = false;
    prevErr = err;
    return kp * err + ki * integral + kd * dFilt;
  }
};
PID linePID = {KP, KI, KD, I_MAX};

float lastErr = 0;
unsigned long lastT = 0;
unsigned long lastSeenMs = 0;

// One control step. Returns true if the line was seen this step.
// err > 0 means the line is to the robot's LEFT, so the robot turns left.
bool FollowLine(int baseSpeed) {
  unsigned long now = micros();
  float dt = (now - lastT) / 1e6f;
  lastT = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.03f;

  float pos;
  if (!LinePosition(pos)) {                    // lost: creep toward where we last saw it
    int s = (lastErr >= 0) ? 1 : -1;           // +1 = turn left
    drive(baseSpeed / 2 - s * 20, baseSpeed / 2 + s * 20);
    return false;
  }
  // fix #1: index 0 is on the robot's right in the sim, so a low pos = line on the right
  float err = SENSOR0_ON_RIGHT ? (pos - CENTER) : (CENTER - pos);
  lastErr = err;
  float turn = linePID.update(err, dt);
  drive(baseSpeed - turn, baseSpeed + turn);  // line left -> right wheel faster -> turn left
  return true;
}

// =============================================================== state machine
enum State { CALIBRATE, FIND_LINE, FOLLOW, CURB_PUSH, BACKUP, TURN_TO_STACK,
             PUSH_STACK, RELEASE, BACK_OFF, DONE };
const char *STATE_NAMES[] = {"CALIBRATE", "FIND_LINE", "FOLLOW", "CURB_PUSH", "BACKUP",
                             "TURN_TO_STACK", "PUSH_STACK", "RELEASE", "BACK_OFF", "DONE"};
State state = CALIBRATE;
unsigned long stateStart = 0;
unsigned long autoStart = 0;
bool autoGo = false;                            // fix #4: set in setup()

void enter(State s) {
  state = s;
  stateStart = millis();
  Serial.printf("[AUTO] %5.2f s  -> %s\n", (millis() - autoStart) / 1000.0, STATE_NAMES[s]);
}
unsigned long inState() { return millis() - stateStart; }

void setup() {
  RedRobot::InitRedRobot();
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();
  // The kit starts AutonomousLoop as soon as the robot boots (training slides), so
  // auto is armed here. To use the "press one button to start auto" rule instead,
  // set autoGo = false and set it to true from one controller button.
  autoGo = true;
}

void AutonomousLoop() {
  if (!autoGo || state == DONE) return;        // DONE: stop writing so teleop isn't fought
  if (autoStart == 0) {
    autoStart = millis();
    paddles(PADDLE_CLOSED_L, PADDLE_CLOSED_R);
    enter(CALIBRATE);
  }

  switch (state) {
    case CALIBRATE:
      drive(0, 0);
      if (inState() >= CALIBRATE_SETTLE_MS) {
        CalibrateLine();                       // sensors must be over plain floor here
        linePID.reset();
        lastT = micros();
        enter(FIND_LINE);
      }
      break;

    case FIND_LINE: {
      float pos;
      drive(FIND_SPEED, FIND_SPEED);
      if (LinePosition(pos)) {
        lastSeenMs = millis();
        enter(FOLLOW);
      } else if (inState() > FIND_LINE_TIMEOUT_MS) {
        Serial.println("[AUTO] no line found - following anyway");
        lastSeenMs = millis();
        enter(FOLLOW);
      }
      break;
    }

    case FOLLOW:
      if (FollowLine(FOLLOW_SPEED)) lastSeenMs = millis();
      if ((inState() > FOLLOW_MIN_MS && millis() - lastSeenMs > LOST_MS) || inState() > FOLLOW_MAX_MS) {
        Serial.printf("[AUTO] end of line (%s)\n", inState() > FOLLOW_MAX_MS ? "timeout" : "line lost");
        enter(CURB_PUSH);
      }
      break;

    case CURB_PUSH:                            // straight on: tip the pipe over the curb
      drive(CURB_PUSH_SPEED, CURB_PUSH_SPEED);
      if (inState() > CURB_PUSH_MS) enter(BACKUP);
      break;

    case BACKUP:
      drive(-DRIVE_POWER, -DRIVE_POWER);
      if (inState() > BACKUP_MS) { drive(0, 0); enter(TURN_TO_STACK); }
      break;

    case TURN_TO_STACK:
      drive(-TURN_POWER, TURN_POWER);          // spin left
      if (inState() > TURN_TO_STACK_MS) { drive(0, 0); enter(PUSH_STACK); }
      break;

    case PUSH_STACK:
      drive(PUSH_POWER, PUSH_POWER);
      if (inState() > STACK_PUSH_MS) { drive(0, 0); enter(RELEASE); }
      break;

    case RELEASE:
      drive(0, 0);
      paddles(PADDLE_OPEN_L, PADDLE_OPEN_R);
      if (inState() > RELEASE_MS) enter(BACK_OFF);
      break;

    case BACK_OFF:
      drive(-PUSH_POWER, -PUSH_POWER);
      if (inState() > BACK_OFF_MS) {
        drive(0, 0);
        enter(DONE);
        Serial.println("[AUTO] done - press X for teleop");
      }
      break;

    case DONE:
      break;
  }
}
