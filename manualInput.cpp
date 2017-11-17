// ManualInput.cpp file

#include "manualInput.h"

#define LX_PIN A0 
#define LY_PIN A1 
#define LD_PIN 16
#define RX_PIN A2 
#define RY_PIN A3 
#define RD_PIN 17

#define MIN_UPDATE_TIME_US 100*1000
#define MIN_UPDATE_FREQ_HZ (1000000/MIN_UPDATE_TIME_US)

#define MAX_ROTATION_SPEED_DEG_PER_SEC 30
#define MAX_ROTATION_PER_UPDATE (MAX_ROTATION_SPEED_DEG_PER_SEC / MIN_UPDATE_FREQ_HZ)


ManualInput::ManualInput(void) :
  _Joystick(LX_PIN, LY_PIN, LD_PIN, RX_PIN, RY_PIN, RD_PIN)
{
  _lastUpdate = 0;
}

void ManualInput::GetJoystickInput(double &setPointYawChange, double &setPointThrust, double &setPointDepth)
{
  // Read values in the orientation of our setup
  int rx = _Joystick.readLX();
  int ry = _Joystick.readLY();
  int ly = _Joystick.readRX();
  int lx = - _Joystick.readRY();
  
  //set motor values based on -100 to 100 scale of values
  //THURST
  setPointThrust = ly;

  //YAW
  unsigned long Now = micros();
  unsigned long timeDiff = Now - _lastUpdate;
  if (timeDiff > MIN_UPDATE_TIME_US) // if its been longer than 10ms
  {
    // TODO read from controls to get the current yaw
    
    //Increment yaw by some reasonable amount
    setPointYawChange = (rx/100.0) * MAX_ROTATION_PER_UPDATE;
    _lastUpdate = Now;
  }

  //DEPTH
  setPointDepth = map(ry, -100, 100, 0, 100);
}

void ManualInput::GetButtonPresses(bool &left, bool &right)
{
  left = _Joystick.readLD();
  right = _Joystick.readRD();
}


