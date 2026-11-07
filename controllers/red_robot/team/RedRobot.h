#pragma once

#include <Arduino.h>

namespace RedRobot {

void InitRedRobot();
void InitDCMotors();
void InitServo();
void InitLineSensors();
void InitAuton();

/**
 * @brief Drive motor 0, 2, or 3 at a certain speed (from -100 to 100). Must call InitDCMotor before using.
 */
void WriteDCMotor(uint8_t d, int16_t speed);

/**
 * @brief Stop ALL motors (0 to 3)
 */
static void Brake();

/**
 * @brief Set servo 0, 1, 2, or 3 to the specified angle (0 to 180). Must call InitServo before using.
 */
void SetServo(uint8_t s, int16_t angle);

/**
 * @brief Read line sensor values
 *
 * Reads the values from the 6 line sensors and stores them in the provided
 * array. The values represent the time taken for each sensor to discharge,
 * which can be used to determine the presence of a line. The values go from
 * left to right when looking at the sensor from the front of the robot.
 *
 * @param values An array of 6 integers to store the sensor readings.
 */
void ReadLineSensor(uint16_t values[6]);

/**
 * @brief Start receiving commands from the remote control and stop the autonomous loop.
 */
void StartRemoteControl();

/**
 * @brief Returns whether or not the robot is set to receive commands from the remote control.
 */
bool IsRemoteControlled();
}

namespace Bluefruit {
    enum button_t {
    UP = 0,
    DOWN = 1, 
    LEFT = 2, 
    RIGHT = 3
  };

  void DPad_Pressed(button_t Button, bool buttonPressedDown);
  void Button1_Pressed(bool buttonPressedDown);
  void Button2_Pressed(bool buttonPressedDown);
  void Button3_Pressed(bool buttonPressedDown);
  void Button4_Pressed(bool buttonPressedDown);
}

namespace RR_Controller {
    enum button_t {
    UP = 0,
    DOWN = 1, 
    LEFT = 2, 
    RIGHT = 3,
    CENTER = 4
  };

  void ButtonX_Pressed(bool buttonPressedDown);
  void ButtonY_Pressed(bool buttonPressedDown);
  void ButtonB_Pressed(bool buttonPressedDown);
  void ButtonA_Pressed(bool buttonPressedDown);
  void DPad_Pressed(button_t Button, bool buttonPressedDown);
  void Joystick1(int8_t x, int8_t y);
  void Joystick2(int8_t x, int8_t y);
}
