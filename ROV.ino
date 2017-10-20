#include <Servo.h>
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

// checkpoint2 stuff

int PORTBR = 13;
int PORTBL = 12;
int PORTBC = 11;
int PORTFR = 9;
int PORTFL = 10;

Servo motorBL;
Servo motorBR;
Servo motorBC;

Servo motorFL;
Servo motorFR;

#define MTR_HIGH 180
#define STOP 90
#define MTR_LOW 0

#define MTR_CONSTRAIN(x) (max(min((x), MTR_HIGH), MTR_LOW))

void initMotors()
{
  motorBR.attach(PORTBR);
  motorBL.attach(PORTBL);
  motorBC.attach(PORTBC);
  motorFR.attach(PORTFR);
  motorFL.attach(PORTFL);

  delay(1000);
  
  motorBR.write(STOP);
  motorBL.write(STOP);
  motorBC.write(STOP);
  motorFR.write(STOP);
  motorFL.write(STOP);
  
  Serial.println("motors initialized!");
}

void outputMotors(double pitch, double roll, double yaw)
{
  int BRval, BLval, BCval, FRval, FLval;
  int gain = 3;

  FRval = MTR_CONSTRAIN(STOP+gain*(pitch/2+roll/2));
  FLval = MTR_CONSTRAIN(STOP+gain*(pitch/2-roll/2));
  BRval = MTR_CONSTRAIN(STOP);
  BLval = MTR_CONSTRAIN(STOP);
  BCval = MTR_CONSTRAIN(STOP-gain*pitch);

  Serial.print(FRval);
  Serial.print("\t");
  Serial.print(FLval);
  Serial.print("\t");
  Serial.println(BCval);
  
  motorFR.write(FRval);
  motorFL.write(FLval);
  motorBL.write(BLval);
  motorBR.write(BRval);
  motorBC.write(BCval);
}

//

Attitude attitude;

void setup() 
{
  Wire.begin();
  Serial.begin(38400);
  
  attitude.init();

  initMotors();
}

void loop() 
{
  double pitch, roll, yaw;
  
  attitude.getUpdatedAxes(&pitch, &roll, &yaw);

  //printAxes(pitch, roll, yaw);

  outputMotors(pitch, roll, yaw);

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
