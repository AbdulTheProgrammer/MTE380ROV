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
  Wire.begin();
  Serial.begin(38400);
  Serial.println("Initialized.");
  PID pitchPID(&(y_axis.input), &(y_axis.output), &(y_axis.setpoint),2,5,1, DIRECT);
  PID rollPID(&(x_axis.input), &(x_axis.output), &(x_axis.setpoint),2,5,1, DIRECT);
  PID yawPID(&(z_axis.input), &(z_axis.output), &(z_axis.setpoint),2,5,1, DIRECT);
  attitude.init();

  delay(50);
}

void loop()
{
  double pitch, roll, yaw;
  double Setpoint, Input, Output;

  attitude.getUpdatedAxes(&pitch, &roll, &yaw);
  y_axis.input = pitch;
  x_axis.input = roll;
  z_axis.input = yaw;

  printAxes(pitch, roll, yaw);

<<<<<<< HEAD
  printAxes(pitch, roll, yaw);
=======
  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();
>>>>>>> Added control loop code.

  delay(10);
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
