//
// ManualInput Class to handle joystick input
//

#ifndef ManualInput_h
#define ManualInput_h

#include "thumbstick.h"
#include "controls.h"

class ManualInput
{
private:
  ThumbStick Joystick;//joystick input
  Controls contr;
  double* inputYaw;//pointer to controller yaw value
  double* inputDepth;//pointer to controller depth value 0 to 100
  double* inputThrust;//pointer to controller yaw value -100 to 100
  unsigned long lastUpdate;
public:
  //constructor takes in controller setPoint struct values for yaw, thrust and depth
  ManualInput(double &setPointYaw, double &setPointThrust, double &setPointDepth);

  //function that reads joystick values and sets controller input values accordingly
  void addJoystickInput();
};

#endif
