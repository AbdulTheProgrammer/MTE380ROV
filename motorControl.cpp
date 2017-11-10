#include "motorControl.h"

#define MOTOR_LIMIT_MARGIN (20)
#define MOTOR_MIN (0 + MOTOR_LIMIT_MARGIN)
#define MOTOR_MAX (180 - MOTOR_LIMIT_MARGIN)
#define MOTOR_NEUTRAL ((MOTOR_MAX + MOTOR_MIN)/2)


#define PID_SAMPLE_TIME_MS (20)
#define PID_OUTPUT_MAX (MOTOR_MAX - MOTOR_NEUTRAL)
#define PID_OUTPUT_MIN (MOTOR_MIN - MOTOR_NEUTRAL)

#define PID_KP 1
#define PID_KI 0
#define PID_KD 0

Orienter::Orienter(void) :   pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 1, 0, 0, DIRECT),
  rollPID(&rollInput, &rollOutput, &rollSetpoint, 1, 0, 0, DIRECT),
  yawPID(&yawInput, &yawOutput, &yawSetpoint, 1, 0, 0, DIRECT)
{
  // Init setpoint (xr), input (x) and output (u) to 0
  pitchSetpoint = pitchInput =  pitchOutput = 0;
  rollSetpoint  = rollInput  = rollOutput   = 0;
  yawSetpoint   = yawInput   = yawOutput    = 0;
}

void Orienter::start(void)
{
  // Does a whole bunch of init currenly, including self-test and calibration
  attitude.init();

  // Set sample time of PID
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_MS);
  rollPID.SetSampleTime(PID_SAMPLE_TIME_MS);
  yawPID.SetSampleTime(PID_SAMPLE_TIME_MS);

  // Set PID output limits
  pitchPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  rollPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  yawPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);

  // Set PID control to automatic
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
}

void Orienter::stabilize(void)
{
  // Get newest IMU data on pitch roll and yaw
  attitude.getUpdatedAxes(&pitchInput, &rollInput, &yawInput);

  // Compute new values from the PID loop (saved in the 
  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();

  // Send PID output values to motors TODO
  Serial.print("PID:\t");
  Serial.print(pitchOutput);
  Serial.print("\t");
  Serial.print(rollOutput);
  Serial.print("\t");
  Serial.println(yawOutput);
}

void Orienter::setOrientation(double pitch, double roll, double yaw)
{
  pitchSetpoint = pitch;
  rollSetpoint = roll;
  yawSetpoint = yaw;
}

