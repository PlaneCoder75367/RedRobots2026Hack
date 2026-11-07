#include "../RedRobot.h"

bool useTankDrive = false;

namespace Bluefruit
{
  /* DPad_Pressed is called everytime a D-Pad button is pressed or released.
   *
   * Only activates after RedRobot::StartRemoteControl() is called.
   * Button is one of {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3}
   */
  void DPad_Pressed(button_t Button, bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled())
      return;

    // values stay the same between calls to DPad_Pressed
    static bool button_pressed[4] = {false, false, false, false};

    button_pressed[Button] = (buttonPressedDown == true);

    // TODO: edit your speed output to the yellow motors below!
    // stop = 0, slow = 50, fast = 100
    // range [-100,100]

    int left_speed = 0;
    int right_speed = 0;

    if (button_pressed[UP])
    { // Forward
      left_speed += 100;
      right_speed += 100;
    }
    if (button_pressed[DOWN])
    { // Backward
      left_speed -= 100;
      right_speed -= 100;
    }
    if (button_pressed[LEFT])
    { // Turn left_speed
      left_speed -= 50;
      right_speed += 50;
    }
    if (button_pressed[RIGHT])
    { // Turn right_speed
      left_speed += 50;
      right_speed -= 50;
    }

    // Clamp and write speeds
    RedRobot::WriteDCMotor(2, constrain(left_speed, -100, 100));
    RedRobot::WriteDCMotor(3, constrain(right_speed, -100, 100));
  }

  void Button1_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled() && buttonPressedDown)
    {
      RedRobot::StartRemoteControl();

      // start remote controlled with the DC motors off
      RedRobot::WriteDCMotor(2, 0);
      RedRobot::WriteDCMotor(3, 0);
    }
  }

  void Button2_Pressed(bool buttonPressedDown)
  {
    // Ex: move an arm when the button is pressed
    if (buttonPressedDown)
    {
      RedRobot::SetServo(0, 180);
      RedRobot::SetServo(1, 180);
      RedRobot::SetServo(2, 180);
      RedRobot::SetServo(3, 180);
    }
    else
    {
      RedRobot::SetServo(0, 0);
      RedRobot::SetServo(1, 0);
      RedRobot::SetServo(2, 0);
      RedRobot::SetServo(3, 0);
    }
  }

  void Button3_Pressed(bool buttonPressedDown)
  {
  }

  void Button4_Pressed(bool buttonPressedDown)
  {
  }
}

namespace RR_Controller
{
  void Joystick1(int8_t x, int8_t y)
  {
    if (useTankDrive)
      RedRobot::WriteDCMotor(2, y);
    else
    {
      int leftMotor = y + x;
      int rightMotor = y - x;

      // Normalize to [-100, 100]
      int maxMagnitude = max(abs(leftMotor), abs(rightMotor));

      if (maxMagnitude > 100)
      {
        leftMotor = leftMotor * 75 / maxMagnitude;
        rightMotor = rightMotor * 75 / maxMagnitude;
      }
      RedRobot::WriteDCMotor(2, leftMotor);
      RedRobot::WriteDCMotor(3, rightMotor);
    }
  }

  void Joystick2(int8_t x, int8_t y)
  {
    if (useTankDrive)
      RedRobot::WriteDCMotor(3, y);
  }

  /* DPad_Pressed is called everytime a D-Pad button is pressed or released.
   *
   * Only activates after RedRobot::StartRemoteControl() is called.
   * Button is one of {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3, CENTER = 4}
   */
  void DPad_Pressed(button_t Button, bool buttonPressedDown)
  {
    // values stay the same between calls to DPad_Pressed
    static bool button_pressed[5] = {false, false, false, false, false};

    button_pressed[Button] = (buttonPressedDown == true);

    // TODO: edit your speed output to the yellow motors below!
    // stop = 0, slow = 50, fast = 100
    // range [-100,100]

    int left_speed = 0;
    int right_speed = 0;

    if (button_pressed[UP])
    { // Forward
      left_speed += 100;
      right_speed += 100;
    }
    if (button_pressed[DOWN])
    { // Backward
      left_speed -= 100;
      right_speed -= 100;
    }
    if (button_pressed[LEFT])
    { // Turn left_speed
      left_speed -= 50;
      right_speed += 50;
    }
    if (button_pressed[RIGHT])
    { // Turn right_speed
      left_speed += 50;
      right_speed -= 50;
    }

    // Clamp and write speeds
    RedRobot::WriteDCMotor(2, constrain(left_speed, -100, 100));
    RedRobot::WriteDCMotor(3, constrain(right_speed, -100, 100));
  }

  void ButtonX_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled() && buttonPressedDown)
    {
      RedRobot::StartRemoteControl();

      // start remote controlled with the DC motors off
      RedRobot::WriteDCMotor(2, 0);
      RedRobot::WriteDCMotor(3, 0);
    }
  }

  void ButtonY_Pressed(bool buttonPressedDown)
  {
    Serial.printf("Y pressed\n");
    if (buttonPressedDown)
      useTankDrive = !useTankDrive;
  }

  void ButtonB_Pressed(bool buttonPressedDown)
  {
  }

  void ButtonA_Pressed(bool buttonPressedDown)
  {
  }
}
