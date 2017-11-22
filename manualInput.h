//
// ManualInput Class to handle joystick input
//

#ifndef ManualInput_h
#define ManualInput_h

#include "thumbstick.h"
#include "controls.h"

typedef enum ButtonWaitMode
{
  ButtonWait_Left,
  ButtonWait_Right,
  ButtonWait_Both,
  ButtonWait_Any,
} ButtonWaitMode;

class ManualInput
{
private:
  ThumbStick _Joystick;//joystick input
  unsigned long _lastUpdate;

public:
  ManualInput(void);

  /*
   * \brief  Reads forom Joysticks and maps to yaw, thrust and depth.
   *
   * \param  setPointYawChange   - Indicates the amount that the current yaw should change.
   *                               Does not take into account current yaw, so the user must calculate
   *                               the final yaw value by doing nextYaw = (currentYaw + setPointYawChange).
   * \param  setPointThrust      - New setpoint for thrust. From -100 to 100.
   * \param  setPointDepthChange - Indicates the amount that the current depth should change.
   *                               Does not take into account current depth, so the user must calculate
   *                               the final depth value by doing nextDepth = (currentDepth + setPointDepthChange).
   */
  void GetJoystickInput(double &setPointYawChange, double &setPointThrust, double &setPointDepthChange);

  /*
   * \brief  Reads button press state from the joysticks.
   */
  void GetButtonState(bool &left, bool &right);

  /*
   * \brief   Waits for the specified buttons to be pressed and released.
   *
   * \details If the specified button(s) are currently pressed, the function will wait for it to be released before
   *          waiting for the specified new press.
   */
  void LoopUntilButtonPressAndRelease(ButtonWaitMode mode);

};

#endif
