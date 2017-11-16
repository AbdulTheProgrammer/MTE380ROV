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
  ThumbStick _Joystick;//joystick input
  Controls _contr;
  unsigned long _lastUpdate;
public:
  ManualInput(void);

  /*
   * \brief  Reads forom Joysticks and maps to yaw, thrust and depth.
   * 
   * \param  setPointYawChange - Indicates the amount that the current yaw should change. 
   *                             Does not take into account current yaw, so the user must calculate
   *                             the final yaw value by doing nextYaw = (currentYaw + setPointYawChange).
   * \param  setPointThrust    - New setpoint for thrust. From -100 to 100.
   * \param  setPointDepth     - New setpoint for depth. From 0 to 100.
   */
  void GetJoystickInput(double &setPointYawChange, double &setPointThrust, double &setPointDepth);
};

#endif
