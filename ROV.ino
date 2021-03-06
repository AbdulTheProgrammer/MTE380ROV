#include "controls.h"
#include "manualInput.h"
/************************************************************
MPU9250_Basic
Basic example sketch for MPU-9250 DMP Arduino Library
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library
This example sketch demonstrates how to initialize the
MPU-9250, and stream its sensor outputs to a serial monitor.
Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0
Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/

static Controls controls;
static ManualInput manInput;

#define LOOP_DELAY_MS 50

void setup()
{
  Orientation initialOrientation;
  
  Wire.begin();
  Serial.begin(38400);

  Serial.println("Initialized. Waiting for button press to calibrate MPU..");
  manInput.LoopUntilButtonPressAndRelease(ButtonWait_Any);
  controls.CalibrateAccelGyro();

  Serial.println("Waiting for button press to calibrate AK8963...");
  manInput.LoopUntilButtonPressAndRelease(ButtonWait_Any);
  controls.CalibrateMagnetometer();

  Serial.println("Waiting for both button presses when ROV is in water..");
  manInput.LoopUntilButtonPressAndRelease(ButtonWait_Both);
  controls.Start();
}

void loop()
{
  bool leftButtonPressed, rightButtonPressed;
  double yawChange = 0, thrust = 0, depthChange = 0;
  SpatialState sp;

  // Stabilize the ROV through the control loop stack
  controls.Stabilize(true, true, true, true);

  // Get manual inputs
  manInput.GetJoystickInput(yawChange, thrust, depthChange);

//  Serial.print(yawChange);
//  Serial.print("\t");
//  Serial.print(thrust);
//  Serial.print("\t");
//  Serial.println(depthChange);

  // Update controls setpoint
  controls.GetCurrentSetpoint(sp);
  sp.yaw   += yawChange;
  sp.thrust = thrust;
  sp.depth  = depthChange*200 + 25; // TODO this should be +=, but since we dont have pressure sensor, we are just sending this value to the motors
  controls.SetDesiredSpatialState(sp);

  // Check e-stop
  manInput.GetButtonPresses(leftButtonPressed, rightButtonPressed);
  if (leftButtonPressed && rightButtonPressed)
  {
    controls.Stop();
    while(1);
  }
  
  delay(LOOP_DELAY_MS);
}
