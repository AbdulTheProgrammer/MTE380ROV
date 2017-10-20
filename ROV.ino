#include "attitude.h"

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

Attitude attitude;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Initialized.");

  attitude.init();

  delay(50);
}

void loop() 
{
  double pitch, roll, yaw;
  
  attitude.getUpdatedAxes(&pitch, &roll, &yaw);

 // printAxes(pitch, roll, yaw);

  delay(100);
}

void printAxes(double pitch, double roll, double yaw)
{
  Serial.print("Pitch: ");
  Serial.print(pitch); 
  Serial.print("\t");
  
  Serial.print("Roll: ");
  Serial.print(roll); 
  Serial.print("\t");

  Serial.print("Yaw: ");
  Serial.print(yaw); 
  Serial.print("\r\n");
}

