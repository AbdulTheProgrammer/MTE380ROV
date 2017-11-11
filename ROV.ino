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
  
  Serial.println("Initialized. Calibrating MPU..");
  controls.CalibateMPU9250();

  Serial.println("Calibrating AK8963...");
  controls.CalibateAK8963();

  delay(LOOP_DELAY_MS);
}

void loop()
{
  controls.Stabilize();

  // update controls.setDesiredOrientation() here
  delay(LOOP_DELAY_MS);
}
