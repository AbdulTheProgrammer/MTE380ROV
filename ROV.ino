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

#define LOOP_DELAY_MS 1

void setup()
{
  Orientation initialOrientation;
  
  Wire.begin();
  Serial.begin(38400);

  //controls.CalibrateMotors();

  Serial.println("Initialized. Waiting for button press to calibrate MPU..");
  manInput.LoopUntilButtonPressAndRelease(ButtonWait_Any);
  controls.CalibrateAccelGyro();

  //Serial.println("Waiting for button press to calibrate AK8963...");
  //manInput.LoopUntilButtonPressAndRelease(ButtonWait_Any);
  //controls.CalibrateMagnetometer();

  Serial.println("Waiting for button presses when ROV is in water, to calibrate pressure...");
  manInput.LoopUntilButtonPressAndRelease(ButtonWait_Any);
  controls.CalibratePressureSensor();

  Serial.println("Waiting for both button presses to start ROV controls.");
  manInput.LoopUntilButtonPressAndRelease(ButtonWait_Both);
  controls.Start();
}

void loop()
{
  bool leftButtonPressed, rightButtonPressed;
  double yawChange = 0, thrust = 0, depthChange = 0;
  SpatialState sp;

  // Stabilize the ROV through the control loop stack
  controls.Stabilize(true, true, false, true);

  // Get manual inputs
  manInput.GetJoystickInput(yawChange, thrust, depthChange);

  // Update controls setpoint
  controls.GetCurrentSetpoint(sp);
  sp.yaw   += yawChange;
  sp.thrust = thrust;
  sp.depth += depthChange;
  controls.SetDesiredSpatialState(sp);

  // Check e-stop
  manInput.GetButtonState(leftButtonPressed, rightButtonPressed);
  if (leftButtonPressed && rightButtonPressed)
  {
    controls.Stop();
    while(1);
  }
  else if (leftButtonPressed)
  {
    controls.DecreaseTuning();
    delay(500);
  }
  else if (rightButtonPressed)
  {
    controls.IncreaseTuning();
    delay(500);
  }
  
  delay(LOOP_DELAY_MS);
}
