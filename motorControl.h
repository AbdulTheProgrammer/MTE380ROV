#pragma once

#include <PID_v1.h>
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
  /* motor stuff */
  PID pitchPID;
  PID rollPID;
  PID yawPID;
  Attitude attitude;
  /* motor stuff */
 public:
  Orienter();
  ~Orienter();
  void step();
  void setOrientation(double pitch, double roll, double yaw);
  void updateOrientation():
};
