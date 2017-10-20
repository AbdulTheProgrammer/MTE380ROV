static double pitchInput = 0, rollInput = 0, yawInput = 0;
static double pitchOutput = 0, rollOutput = 0, yawOutput = 0;
static double pitchCommand = 0, rollCommand = 0, yawCommand = 0;

Orienter::Orienter()
{
  // Does a whole bunch of init currenly, including self-test and calibration
  attitude.init();

  // init the ps2 controller

  // init the PID
  pitchPID = PID(&pitchInput, &pitchOutput, &pitchCommand, Kp, Ki, Kd, DIRECT);
  rollPID = PID(&rollInput, &rollOutput, &rollCommand, Kp, Ki, Kd, DIRECT);
  yawPID = PID(&yawInput, &yawOutput, &yawCommand, Kp, Ki, Kd, DIRECT);
  
  // set PID sample time and saturation
  //pitchPID = ;
  //rollPID = ;
  //yawPID = ;

}

void Orienter::updateOrientation()
{
  attitude.getUpdatedAxes( &pitchInput, &rollInput, &yawInput);
  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();
  /* Send pitchOutput to motor controller */

}

void Orienter::setOrientation(double pitch, double roll, double yaw)
{
  pitchCommand = pitch;
  rollCommand = roll;
  yawCommand = yaw;

}

