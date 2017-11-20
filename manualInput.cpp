// ManualInput.cpp file

#include "manualInput.h"

#define LX_PIN A0 
#define LY_PIN A1 
#define LD_PIN 16
#define RX_PIN A2 
#define RY_PIN A3 
#define RD_PIN 17

#define USEC_PER_SEC (1000000)
#define DEBOUNCE_TIME_MS (50)

#define MIN_UPDATE_TIME_US (100*1000UL)
#define MIN_UPDATE_FREQ_HZ (USEC_PER_SEC/MIN_UPDATE_TIME_US)

#define MAX_ROTATION_SPEED_DEG_PER_SEC (15)
#define MAX_ROTATION_PER_UPDATE ((double)MAX_ROTATION_SPEED_DEG_PER_SEC / MIN_UPDATE_FREQ_HZ)

#define MAX_DEPTH_CHANGE_PER_SEC (50)
#define MAX_DEPTH_CHANGE_PER_UPDATE ((double)MAX_DEPTH_CHANGE_PER_SEC / MIN_UPDATE_FREQ_HZ)


ManualInput::ManualInput(void) :
  _Joystick(LX_PIN, LY_PIN, LD_PIN, RX_PIN, RY_PIN, RD_PIN)
{
  _lastUpdate = 0;
}

void ManualInput::GetJoystickInput(double &setPointYawChange, double &setPointThrust, double &setPointDepthChange)
{
  // Read values in the orientation of our setup
  int lx = _Joystick.readLX();
  int ly = -_Joystick.readLY();
  int ry = _Joystick.readRX();
  int rx = _Joystick.readRY();

  // Get time difference between past call to this function
  unsigned long Now = micros();
  unsigned long timeDiff = Now - _lastUpdate;
  
  // Read thrust straight from the joystick
  setPointThrust = ly;

  // Increment yaw and depth
  setPointYawChange   = (((double)rx/THUMBSTICK_ANALOG_OUTPUT_MAX) * MAX_ROTATION_SPEED_DEG_PER_SEC) * ((double)timeDiff / USEC_PER_SEC);
  setPointDepthChange = (((double)ry/THUMBSTICK_ANALOG_OUTPUT_MAX) * MAX_DEPTH_CHANGE_PER_SEC) * ((double)timeDiff / USEC_PER_SEC);

  // Log update time
  _lastUpdate = Now;
}

void ManualInput::GetButtonPresses(bool &left, bool &right)
{
  left = _Joystick.readLD();
  right = _Joystick.readRD();
  delay(DEBOUNCE_TIME_MS);
}

void ManualInput::LoopUntilButtonPressAndRelease(ButtonWaitMode mode)
{
  // First wait for both buttons to be released
  while (_Joystick.readLD() || _Joystick.readRD());
  delay(DEBOUNCE_TIME_MS);
  
  switch (mode)
  {
    case ButtonWait_Left:
    {
      while (_Joystick.readLD());
      delay(DEBOUNCE_TIME_MS);
      while (!_Joystick.readLD());
      delay(DEBOUNCE_TIME_MS);
      while (_Joystick.readLD());
      delay(DEBOUNCE_TIME_MS);
      break;
    }
    case ButtonWait_Right:
    {
      while (_Joystick.readRD());
      delay(DEBOUNCE_TIME_MS);
      while (!_Joystick.readRD());
      delay(DEBOUNCE_TIME_MS);
      while (_Joystick.readRD());
      delay(DEBOUNCE_TIME_MS);
      break;
    }
    case ButtonWait_Both:
    {
      while (_Joystick.readLD() || _Joystick.readRD());
      delay(DEBOUNCE_TIME_MS);
      while (!_Joystick.readLD() || !_Joystick.readRD());
      delay(DEBOUNCE_TIME_MS);
      while (_Joystick.readLD() || _Joystick.readRD());
      delay(DEBOUNCE_TIME_MS);
      break;
    }
    case ButtonWait_Any:
    {
      while (_Joystick.readLD() || _Joystick.readRD());
      delay(DEBOUNCE_TIME_MS);
      while (!_Joystick.readLD() && !_Joystick.readRD());
      delay(DEBOUNCE_TIME_MS);
      while (_Joystick.readLD() || _Joystick.readRD());
      delay(DEBOUNCE_TIME_MS);
      break;
    }
  }
}


