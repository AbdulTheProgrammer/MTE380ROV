#pragma once

#include <PID_v1.h>

typedef struct axis
{
  double input = 0;
  double output = 0 ;
  double setpoint = 0;
} axis_t;

class motorControl
{
private:
/* */
PID pitchPID;
PID rollPID
PID yawPID;
axis_t x_axis;
axis_t y_axis;
axis_t z_axis;

/* motor stuff */
public:

void update();

}
