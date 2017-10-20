#include "motorControl.h"

static double pitchInput = 0, rollInput = 0, yawInput = 0;
static double pitchOutput = 0, rollOutput = 0, yawOutput = 0;
static double pitchCommand = 0, rollCommand = 0, yawCommand = 0;

Orienter::Orienter(int a) :   pitchPID(&pitchInput, &pitchOutput, &pitchCommand, 1, 0, 0, DIRECT),
  rollPID(&rollInput, &rollOutput, &rollCommand, 1, 0, 0, DIRECT),
  yawPID(&yawInput, &yawOutput, &yawCommand, 1, 0, 0, DIRECT)
{
  // Does a whole bunch of init currenly, including self-test and calibration
  attitude.init();
}

void Orienter::updateOrientation()
{
  attitude.getUpdatedAxes( &pitchInput, &rollInput, &yawInput);
  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();
  /* Send pitchOutput to motor controller */

  Serial.print("PID:\t");
  Serial.print(pitchOutput);
  Serial.print("\t");
  Serial.print(rollOutput);
  Serial.print("\t");
  Serial.println(yawOutput);

}

void Orienter::setOrientation(double pitch, double roll, double yaw)
{
  pitchCommand = pitch;
  rollCommand = roll;
  yawCommand = yaw;

}

