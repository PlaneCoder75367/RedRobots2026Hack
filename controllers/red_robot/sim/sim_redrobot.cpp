// Webots implementation of the Red Robot 2026 "RedRobot.h" API.
//
// It replaces src/RedRobot.cpp + src/ble.h from RR26_Arduino, so the team's
// RR26_Arduino.ino and src/RemoteControl.cpp compile UNCHANGED and drive the
// simulated robot. Behaviour mirrors the firmware:
//   * AutonomousLoop() runs repeatedly until RedRobot::StartRemoteControl()
//   * after that, WriteDCMotor/SetServo calls coming from AutonomousLoop are ignored
//     (only controller callbacks can move the robot), like the firmware's task check
//   * motor 1 is rejected (pin shared with the line-sensor LED on the real board)
//   * SetServo maps -180..180 -> 500..2500 us, so on a standard 180 deg servo
//     SetServo(0) = centre, SetServo(180) = one end, SetServo(-180) = the other
//   * ReadLineSensor returns "discharge time" style values (white ~150, black ~2500)
//     and takes simulated time to run, like the real RC-discharge read
//
// Controller (Webots 3D view must have keyboard focus - click it first):
//   Arrows = D-pad, Space = D-pad centre, WASD = left joystick, IJKL = right joystick
//   X or 1 = X (start teleop in the sample code), Y or 2 = Y, 3 = A, B or 4 = B
//   A USB gamepad also works if Webots detects it.

#include <webots/DistanceSensor.hpp>
#include <webots/Joystick.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>

#include <atomic>
#include <random>
#include <set>
#include <string>

#include "Arduino.h"
#include "../team/RedRobot.h"
#include "../team/CONFIGURE-ME.h"

extern void setup();
extern void AutonomousLoop();

SimSerial Serial;

namespace {
webots::Robot *R = nullptr;
int TS = 8;
webots::Motor *dc[4] = {nullptr, nullptr, nullptr, nullptr};
webots::Motor *servo[4] = {nullptr, nullptr, nullptr, nullptr};
webots::DistanceSensor *lf[6] = {};
webots::Receiver *rx = nullptr;
webots::Keyboard *kb = nullptr;
webots::Joystick *gp = nullptr;

enum Phase { PRE, AUTO, TELEOP, END, PRACTICE };
Phase phase = PRE;
const char *PHASE_NAMES[] = {"PRE", "AUTO", "TELEOP", "END", "PRACTICE"};

std::atomic<bool> in_remote_control{false};
bool in_callback = false;     // true while dispatching controller callbacks
bool use_keyboard = false;
bool dc_init = false, servo_init = false, lf_init = false;
bool warned_dc = false, warned_servo = false, warned_lf = false, warned_gpio = false, warned_auto_input = false;
double max_wheel_speed = 26.0;
bool sim_debug = std::getenv("RR_SIM_DEBUG") != nullptr;  // set RR_SIM_DEBUG=1 to log every motor command

// line sensor model
const double RAW_WHITE = 1000.0;   // DistanceSensor value over the light grey floor
const double DEADBAND = 6.0;       // % duty below which a TT motor doesn't turn
std::mt19937 rng(12345);
std::normal_distribution<double> noise(0.0, 1.0);

void handleInputs();
void pollReceiver();

void simStep() {
  if (R->step(TS) == -1) {
    delete R;
    std::exit(0);
  }
  pollReceiver();
  if (phase == END) {
    for (auto *m : dc)
      if (m) m->setVelocity(0);
  }
  handleInputs();
}

void setPhase(Phase p) {
  if (p == phase) return;
  phase = p;
  std::printf("[SIM] match phase -> %s\n", PHASE_NAMES[p]);
  std::fflush(stdout);
}

void pollReceiver() {
  while (rx && rx->getQueueLength() > 0) {
    std::string msg(static_cast<const char *>(rx->getData()), rx->getDataSize());
    rx->nextPacket();
    if (msg.rfind("PHASE:", 0) != 0) continue;
    std::string p = msg.substr(6);
    if (p == "PRE") setPhase(PRE);
    else if (p == "AUTO") setPhase(AUTO);
    else if (p == "TELEOP") setPhase(TELEOP);
    else if (p == "END") setPhase(END);
    else if (p == "PRACTICE") setPhase(PRACTICE);
  }
}

// ------------------------------------------------------------- controller input
struct Pad {
  bool dpad[5] = {false, false, false, false, false};
  bool btn[4] = {false, false, false, false};  // X, Y, A, B
  int j1x = 0, j1y = 0, j2x = 0, j2y = 0;
};
Pad last;

void dispatch(const Pad &now) {
  in_callback = true;
  const RR_Controller::button_t dirs[5] = {RR_Controller::UP, RR_Controller::DOWN, RR_Controller::LEFT,
                                          RR_Controller::RIGHT, RR_Controller::CENTER};
  for (int i = 0; i < 5; i++)
    if (now.dpad[i] != last.dpad[i]) RR_Controller::DPad_Pressed(dirs[i], now.dpad[i]);
  if (now.btn[0] != last.btn[0]) RR_Controller::ButtonX_Pressed(now.btn[0]);
  if (now.btn[1] != last.btn[1]) RR_Controller::ButtonY_Pressed(now.btn[1]);
  if (now.btn[2] != last.btn[2]) RR_Controller::ButtonA_Pressed(now.btn[2]);
  if (now.btn[3] != last.btn[3]) RR_Controller::ButtonB_Pressed(now.btn[3]);
  if (now.j1x != last.j1x || now.j1y != last.j1y) RR_Controller::Joystick1((int8_t)now.j1x, (int8_t)now.j1y);
  if (now.j2x != last.j2x || now.j2y != last.j2y) RR_Controller::Joystick2((int8_t)now.j2x, (int8_t)now.j2y);
  in_callback = false;
  last = now;
}

int axis(int raw) {  // gamepad axis (-32768..32767) -> -100..100 with a small deadzone
  int v = raw * 100 / 32767;
  if (std::abs(v) < 8) v = 0;
  return constrain(v, -100, 100);
}

void handleInputs() {
  if (!use_keyboard) return;
  Pad now;
  std::set<int> keys;
  for (int k = kb->getKey(); k != -1; k = kb->getKey()) keys.insert(k & webots::Keyboard::KEY);
  static std::set<int> lastKeys;
  if (sim_debug && keys != lastKeys) {
    std::printf("[SIM] keys:");
    for (int k : keys) std::printf(" %d", k);
    std::printf("\n");
    lastKeys = keys;
  }
  auto down = [&](int k) { return keys.count(k) > 0; };
  now.dpad[0] = down(webots::Keyboard::UP);
  now.dpad[1] = down(webots::Keyboard::DOWN);
  now.dpad[2] = down(webots::Keyboard::LEFT);
  now.dpad[3] = down(webots::Keyboard::RIGHT);
  now.dpad[4] = down(' ');
  // controller buttons: number keys, or the letter itself where it doesn't
  // clash with WASD (A is only on 3)
  now.btn[0] = down('1') || down('X');
  now.btn[1] = down('2') || down('Y');
  now.btn[2] = down('3');
  now.btn[3] = down('4') || down('B');
  now.j1x = (down('D') ? 100 : 0) - (down('A') ? 100 : 0);
  now.j1y = (down('W') ? 100 : 0) - (down('S') ? 100 : 0);
  now.j2x = (down('L') ? 100 : 0) - (down('J') ? 100 : 0);
  now.j2y = (down('I') ? 100 : 0) - (down('K') ? 100 : 0);

  if (gp && gp->isConnected()) {
    int n = gp->getNumberOfAxes();
    if (n >= 2 && (now.j1x == 0 && now.j1y == 0)) { now.j1x = axis(gp->getAxisValue(0)); now.j1y = -axis(gp->getAxisValue(1)); }
    if (n >= 4 && (now.j2x == 0 && now.j2y == 0)) { now.j2x = axis(gp->getAxisValue(2)); now.j2y = -axis(gp->getAxisValue(3)); }
    for (int b = gp->getPressedButton(); b != -1; b = gp->getPressedButton()) {
      // common Xbox-style numbering: 0=A 1=B 2=X 3=Y (varies by OS/pad)
      if (b == 0) now.btn[2] = true;
      if (b == 1) now.btn[3] = true;
      if (b == 2) now.btn[0] = true;
      if (b == 3) now.btn[1] = true;
    }
  }

  const bool allowed = (phase == TELEOP || phase == PRACTICE);
  if (!allowed) {
    bool any = keys.size() > 0;
    if (any && phase == AUTO && !warned_auto_input) {
      std::printf("[SIM] Controller input ignored during autonomous (hands off the controller!)\n");
      warned_auto_input = true;
    }
    last = Pad();  // so held keys register as a fresh press once teleop begins
    return;
  }
  dispatch(now);
}

void hardStop() {
  for (auto *m : dc)
    if (m) m->setVelocity(0);
}
}  // namespace

// ------------------------------------------------------------- Arduino timing / GPIO
void delay(unsigned long ms) {
  const double end = R->getTime() + ms / 1000.0 - 1e-9;
  while (R->getTime() < end) simStep();
}
void delayMicroseconds(unsigned int us) { delay((us + 999) / 1000); }
unsigned long millis() { return (unsigned long)(R->getTime() * 1000.0); }
unsigned long micros() { return (unsigned long)(R->getTime() * 1e6); }
static void gpioWarn() {
  if (!warned_gpio) {
    std::printf("[SIM] note: raw GPIO (pinMode/digitalWrite/analogRead...) does nothing in the simulator\n");
    warned_gpio = true;
  }
}
void pinMode(int, int) {}
void digitalWrite(int, int) { gpioWarn(); }
int digitalRead(int) { gpioWarn(); return 0; }
int analogRead(int) { gpioWarn(); return 0; }
void analogWrite(int, int) { gpioWarn(); }

// ------------------------------------------------------------- RedRobot API
namespace RedRobot {

void InitRedRobot() {
  std::printf("[INIT] Red Robot sim: RR-%d-%s as %s\n", TEAM_NUMBER, TEAM_NAME, R->getName().c_str());
}
void InitDCMotors() { dc_init = true; std::printf("[DC_MOTOR] Initializing channels...\n"); }
void InitServo() { servo_init = true; std::printf("[SERVO] Initializing channels...\n"); }
void InitLineSensors() { lf_init = true; }
void InitAuton() {
  in_remote_control.store(false);
  std::printf("[INIT] AutonomousLoop will run from the Arduino loop task\n");
}

void WriteDCMotor(uint8_t d, int16_t speed) {
  if (in_remote_control.load() && !in_callback) return;  // auto code silenced in teleop
  if (phase == END) return;
  if (d >= 4 || d == 1 || !dc[d]) {
    std::printf("[DC_MOTOR] Error: Invalid motor index %d\n", d);
    return;
  }
  if (!dc_init) {
    if (!warned_dc) std::printf("[SIM] WriteDCMotor called before InitDCMotors() - ignored (same as hardware)\n");
    warned_dc = true;
    return;
  }
  speed = constrain(speed, (int16_t)-100, (int16_t)100);
  double mag = std::abs(speed) < DEADBAND ? 0.0 : std::abs(speed) / 100.0;
  dc[d]->setVelocity((speed < 0 ? -1.0 : 1.0) * mag * max_wheel_speed);
  if (sim_debug) std::printf("[SIM] t=%.3f motor%d <- %d\n", R->getTime(), d, speed);
}

static void Brake() {
  for (uint8_t i = 0; i < 4; i++)
    if (i != 1) WriteDCMotor(i, 0);
}

void SetServo(uint8_t s, int16_t angle) {
  if (in_remote_control.load() && !in_callback) return;
  if (phase == END) return;
  if (s >= 4 || !servo[s]) return;
  if (!servo_init) {
    if (!warned_servo) std::printf("[SIM] SetServo called before InitServo() - ignored (same as hardware)\n");
    warned_servo = true;
    return;
  }
  angle = constrain(angle, (int16_t)-180, (int16_t)180);
  // firmware: -180..180 -> 500..2500 us; a 180 deg servo spans 500..2500 us,
  // so physical offset from centre = angle / 2 degrees
  double pos = (angle / 2.0) * DEG_TO_RAD;
  servo[s]->setPosition(constrain(pos, -1.5708, 1.5708));
}

void ReadLineSensor(uint16_t values[6]) {
  if (!lf_init && !warned_lf) {
    std::printf("[SIM] ReadLineSensor called before InitLineSensors()\n");
    warned_lf = true;
  }
  unsigned long total_us = 7000;  // LED settle + 1 ms charge per sensor (vTaskDelay(1) x 7)
  for (int i = 0; i < 6; i++) {
    double refl = constrain(lf[i]->getValue() / RAW_WHITE, 0.0, 1.0);
    double us = 150.0 + (1.0 - refl) * 2350.0;
    us *= 1.0 + 0.03 * noise(rng);
    us = constrain(us, 0.0, 10000.0);
    values[i] = (uint16_t)us;
    total_us += values[i];
  }
  delay((total_us + 999) / 1000);  // the real read blocks this long
}

void StartRemoteControl() {
  in_remote_control.store(true);
  std::printf("[SIM] Remote control started\n");
}
bool IsRemoteControlled() { return in_remote_control.load(); }

}  // namespace RedRobot

// weak defaults (the team's RemoteControl.cpp overrides whichever it defines)
namespace Bluefruit {
void __attribute__((weak)) DPad_Pressed(button_t, bool) {}
void __attribute__((weak)) Button1_Pressed(bool) {}
void __attribute__((weak)) Button2_Pressed(bool) {}
void __attribute__((weak)) Button3_Pressed(bool) {}
void __attribute__((weak)) Button4_Pressed(bool) {}
}  // namespace Bluefruit
namespace RR_Controller {
void __attribute__((weak)) ButtonX_Pressed(bool) {}
void __attribute__((weak)) ButtonY_Pressed(bool) {}
void __attribute__((weak)) ButtonB_Pressed(bool) {}
void __attribute__((weak)) ButtonA_Pressed(bool) {}
void __attribute__((weak)) DPad_Pressed(button_t, bool) {}
void __attribute__((weak)) Joystick1(int8_t, int8_t) {}
void __attribute__((weak)) Joystick2(int8_t, int8_t) {}
}  // namespace RR_Controller

// ------------------------------------------------------------- main
int main() {
  R = new webots::Robot();
  TS = (int)R->getBasicTimeStep();

  const char *dcNames[4] = {"motor0", nullptr, "motor2", "motor3"};
  for (int i = 0; i < 4; i++)
    if (dcNames[i]) {
      dc[i] = R->getMotor(dcNames[i]);
      dc[i]->setPosition(INFINITY);
      dc[i]->setVelocity(0);
      max_wheel_speed = dc[i]->getMaxVelocity();
    }
  for (int i = 0; i < 4; i++) servo[i] = R->getMotor("servo" + std::to_string(i));
  for (int i = 0; i < 6; i++) {
    lf[i] = R->getDistanceSensor("lf" + std::to_string(i));
    lf[i]->enable(TS);
  }
  rx = R->getReceiver("match_rx");
  rx->enable(TS);

  use_keyboard = R->getCustomData().find("keyboard") != std::string::npos;
  if (use_keyboard) {
    kb = R->getKeyboard();
    kb->enable(TS);
    gp = R->getJoystick();
    gp->enable(TS);
    std::printf("[SIM] %s reads the keyboard: arrows=D-pad, WASD=left stick, IJKL=right stick, X/1=X Y/2=Y 3=A B/4=B\n",
                R->getName().c_str());
  }

  setup();  // the team's setup()

  // wait for the game supervisor to start the match (or fall back to practice)
  const double t0 = R->getTime();
  while (phase == PRE) {
    simStep();
    if (R->getTime() - t0 > 2.0 && phase == PRE) {
      std::printf("[SIM] no game supervisor found - practice mode\n");
      setPhase(PRACTICE);
    }
  }

  // Arduino loop(): AutonomousLoop() until remote control starts
  while (true) {
    if (phase == END) {
      hardStop();
      simStep();
      continue;
    }
    if (!in_remote_control.load()) AutonomousLoop();
    simStep();
  }
}
