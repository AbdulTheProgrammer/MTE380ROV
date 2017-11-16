// ManualInput.cpp file

#include "manualInput.h"

#define LX_PIN 1
#define LY_PIN 2
#define LD_PIN 3
#define RX_PIN 4
#define RY_PIN 5
#define RD_PIN 6

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
  int lx = _Joystick.readLX();
  int ly = _Joystick.readLY();
  int rx = _Joystick.readRX();
  int ry = _Joystick.readRY();

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
