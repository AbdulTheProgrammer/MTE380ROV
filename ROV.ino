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
  controls.InitializeMotors();

  // TODO some button press to begin 5- second MPU calibration
  
  Serial.println("Initialized. Calibrating MPU..");
  controls.CalibrateAccelGyro();

  // TODO some button press to begin 15-second magnetometer calibration
  
  //Serial.println("Calibrating AK8963...");
  //controls.CalibrateMagnetometer();

  SpatialState sp;

  sp.pitch = 0;
  sp.roll = 0;
  sp.yaw = 0;
  sp.thrust = 0;
  sp.depth = 25; // Experimental value that keeps it neutrally buoyant

  controls.SetDesiredSpatialState(sp);

  delay(LOOP_DELAY_MS);

  //Serial.println("Waiting 10 sec to be put into water...");
  //delay(10000);
}

void loop()
{
  // TODO currently this prints out values sent to motors. Set PRINT_MOTOR_VALUES to 0 in controls.h to turn off.
  //controls.Stabilize(true, true, false, true);

  // TODO update controls.setDesiredSpatialState() here, according to either
  // thumbstick input or route planning data

  double yawChange = 0, thrust = 0, depth = 0;
  
  manInput.GetJoystickInput(yawChange, thrust, depth);

//  Serial.print(yawChange);
//  Serial.print("\t");
//  Serial.print(thrust);
//  Serial.print("\t");
//  Serial.println(depth);
  
  delay(LOOP_DELAY_MS);
}
