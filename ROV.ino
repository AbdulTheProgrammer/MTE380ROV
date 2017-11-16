#include "controls.h"
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

#define LOOP_DELAY_MS 50

void setup()
{
  Orientation initialOrientation;
  
  Wire.begin();
  Serial.begin(38400);
  controls.InitializeMotors();

  // TODO some button press to begin 5- second MPU calibration
  
  Serial.println("Initialized. Calibrating MPU..");
  controls.CalibrateAccelGyro();

  // TODO some button press to begin 15-second magnetometer calibration
  
  Serial.println("Calibrating AK8963...");
  controls.CalibrateMagnetometer();

  delay(LOOP_DELAY_MS);
}

void loop()
{
  // TODO currently this prints out values sent to motors. Set PRINT_MOTOR_VALUES to 0 in controls.h to turn off.
  controls.Stabilize(true, true, true, false);

  // TODO update controls.setDesiredSpatialState() here, according to either
  // thumbstick input or route planning data
  
  delay(LOOP_DELAY_MS);
}
