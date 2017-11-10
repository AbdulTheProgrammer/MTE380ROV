#pragma once

#include <PID_v1.h>
#include <Servo.h>
#include "attitude.h"

typedef struct axis
{
  double input = 0;
  double output = 0 ;
  double setpoint = 0;
} axis_t;

class Orienter
{
 private: 
  // PID controllers
  PID pitchPID;
  PID rollPID;
  PID yawPID;
  
  // PID setpoint (xr), input (x), and output (u)
  double pitchSetpoint, rollSetpoint, yawSetpoint;
  double pitchInput, rollInput, yawInput;
  double pitchOutput, rollOutput, yawOutput;

  // IMU instance
  Attitude attitude;

  // Motor instances
  Servo motorBL;
  Servo motorBR;
  Servo motorBC;
  Servo motorFL;
  Servo motorFR;

  void setMotors(void);
 public:
  Orienter(void);
  void start(void);
  void setOrientation(double pitch, double roll, double yaw);
  void stabilize(void);
};
