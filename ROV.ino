#include "attitude.h"
#include "motorControl.h"
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
Orienter orienter;

#define LOOP_DELAY_MS 500

void setup()
{
  Wire.begin();
  Serial.begin(38400);
  Serial.println("Initialized.");
  orienter.start();

  delay(LOOP_DELAY_MS);
}

void loop()
{
  orienter.stabilize();

  

  delay(LOOP_DELAY_MS);
}
