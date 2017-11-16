// ManualInput.cpp file

#include "manualInput.h"

#define LX_PIN 1
#define LY_PIN 2
#define LD_PIN 3

#define RX_PIN 4
#define RY_PIN 5
#define RD_PIN 6

#define MAX_SPEED 180
#define NEUTRAL_SPEED 90
#define MIN_SPEED 0

#define MIN_UPDATE_TIME_US 100*1000
#define MIN_UPDATE_FREQ_HZ (1000000/MIN_UPDATE_TIME_US)

#define MAX_ROTATION_SPEED_DEG_PER_SEC 30
#define MAX_ROTATION_PER_UPDATE (MAX_ROTATION_SPEED_DEG_PER_SEC / MIN_UPDATE_FREQ_HZ)


ManualInput::ManualInput(double &setPointYaw, double &setPointThrust, double &setPointDepth)
{
  inputYaw = setPointYaw;
  inputDepth = setPointDepth;
  inputThrust = setPointThrust;
  Joystick(LX_PIN, LY_PIN, LD_PIN, RX_PIN, RY_PIN, RD_PIN);
  lastUpdate = 0;
}

void ManualInput::addJoystickInput(void)
{
  int lx = Joystick.readLX();
  int ly = Joystick.readLY();
  int rx = Joystick.readRX();
  int ry = Joystick.readRY();

  //set motor values based on -100 to 100 scale of values
  //THURST
  *inputThrust = ly;

  //YAW
  unsigned long Now = micros();
  unsigned long timeDiff = Now - lastUpdate;
  if (timeDiff > MIN_UPDATE_TIME_US) // if its been longer than 10ms
  {
    //Increment yaw by some reasonable
    *inputYaw += (rx/100.0) * MAX_ROTATION_PER_UPDATE;
    lastUpdate = Now;
  }

  //DEPTH
  *inputDepth = map(ry, -100, 100, 0, 100);

}
