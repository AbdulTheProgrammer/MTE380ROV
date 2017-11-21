#include "controls.h"

#define MOTOR_STARTUP_DELAY_MS (2000)
#define MOTOR_LIMIT_MARGIN     (20)
#define MOTOR_MIN              (0 + MOTOR_LIMIT_MARGIN)
#define MOTOR_MAX              (180 - MOTOR_LIMIT_MARGIN)
#define MOTOR_NEUTRAL          ((MOTOR_MAX + MOTOR_MIN)/2)
#define MOTOR_CONSTRAIN(X)     (min(max((X), MOTOR_MIN), MOTOR_MAX))

//TODO tune these later
#define MIN_DEPTH 0
#define MAX_DEPTH 2000

#define PID_SAMPLE_TIME_MS (20)
#define PID_OUTPUT_MAX     (MOTOR_MAX - MOTOR_NEUTRAL)
#define PID_OUTPUT_MIN     (MOTOR_MIN - MOTOR_NEUTRAL)

#define PID_PITCH_KP 1
#define PID_PITCH_KI 0
#define PID_PITCH_KD 0 //0.015 // makes it more stable, but it starts spinning bc the BC motor is tilted. Needs yaw correction.

#define PID_ROLL_KP  1
#define PID_ROLL_KI  0
#define PID_ROLL_KD  0 //0.015 // makes it more stable, but it starts spinning bc the BC motor is tilted. Needs yaw correction.

#define PID_YAW_KP   1
#define PID_YAW_KI   0
#define PID_YAW_KD   0

#define PID_DEPTH_KP   1
#define PID_DEPTH_KI   0
#define PID_DEPTH_KD   0

#define MOTOR_BR_REVERSED 1
#define MOTOR_BL_REVERSED 0
#define MOTOR_BC_REVERSED 1
#define MOTOR_FR_REVERSED 0
#define MOTOR_FL_REVERSED 0

#define PRESSURE_TO_DEPTH_M 0.1019  //0.1222

static int PORTBR = 13;
static int PORTBL = 12;
static int PORTBC = 11;
static int PORTFR = 9;
static int PORTFL = 10;

Controls::Controls(void) :   
  _pitchPID(&_sensorsInput.pitch, &_PIDOutput.pitch, &_setPoint.pitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT),
  _rollPID (&_sensorsInput.roll,  &_PIDOutput.roll,  &_setPoint.roll,  PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  DIRECT),
  _yawPID  (&_sensorsInput.yaw,   &_PIDOutput.yaw,   &_setPoint.yaw,   PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   DIRECT),
  _depthPID(&_sensorsInput.depth, &_PIDOutput.depth, &_setPoint.depth, PID_DEPTH_KP, PID_DEPTH_KI, PID_DEPTH_KD, REVERSE), 
  _pSensor(ADDRESS_HIGH),
  _attitude()
{
  // Init _setPoint (xr), input (x) and output (u) to 0
  _setPoint.pitch = _sensorsInput.pitch = _PIDOutput.pitch  = 0;
  _setPoint.roll  = _sensorsInput.roll  = _PIDOutput.roll   = 0;
  _setPoint.yaw   = _sensorsInput.yaw   = _PIDOutput.yaw    = 0;

  // Set sample time of PID
  _pitchPID.SetSampleTime(PID_SAMPLE_TIME_MS);
  _rollPID.SetSampleTime(PID_SAMPLE_TIME_MS);
  _yawPID.SetSampleTime(PID_SAMPLE_TIME_MS);
  _depthPID.SetSampleTime(PID_SAMPLE_TIME_MS);

  // Set PID output limits
  _pitchPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  _rollPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  _yawPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  _depthPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
}

void Controls::CalibrateAccelGyro(void)
{
  // Calibrate the MPU
  _attitude.calibrateMPU9250();
}

void Controls::CalibrateMagnetometer(void)
{
  _attitude.calibrateAK8963();
}

void Controls::Stabilize(bool inStabilizePitch, bool inStabilizeRoll, bool inStabilizeYaw, bool inStabilizeDepth)
{
  GetSensorsInput();

  CalculatePIDs(inStabilizePitch, inStabilizeRoll, inStabilizeYaw, inStabilizeDepth);

  SetNewMotorValues();
}

void Controls::GetSensorsInput(void)
{
  Orientation IMUOrientation;
  
  // Get newest IMU data on pitch roll and yaw
  _attitude.getUpdatedOrientation(IMUOrientation);
  
  // Update internal values for our Spatial State
  _sensorsInput.pitch = IMUOrientation.pitch;
  _sensorsInput.roll  = IMUOrientation.roll;
  _sensorsInput.yaw   = IMUOrientation.yaw;
  _sensorsInput.depth = GetDepth();

  // Note that we have no sensors for thrust
}

void Controls::CalculatePIDs(bool inStabilizePitch, bool inStabilizeRoll, bool inStabilizeYaw, bool inStabilizeDepth)
{
  // Compute new values from the PID loop, saved automatically in the _PIDOutput struct
  if (inStabilizePitch)
  {
    _pitchPID.Compute();
  }
  else
  {
    _PIDOutput.pitch = 0;
  }

  if (inStabilizeRoll)
  {
    _rollPID.Compute();
  }
  else
  {
    _PIDOutput.roll = 0;
  }

  if (inStabilizeYaw)
  {
    _yawPID.Compute();
  }
  else
  {
    _PIDOutput.yaw = 0;
  }

  if(inStabilizeDepth)
  {
    _depthPID.Compute();
  }
  else
  {
    _PIDOutput.depth = 0;
  }

}

void Controls::SetDesiredSpatialState(SpatialState &inSpatialState)
{
  _setPoint = inSpatialState;

  Serial.print("Set Yaw: ");
  Serial.print(_setPoint.yaw);
  Serial.print("\tDepth: ");
  Serial.print(_setPoint.depth);
  Serial.print("\tThrust: ");
  Serial.print(_setPoint.thrust);
  Serial.print("\t");
}

void Controls::GetCurrentSpatialState(SpatialState &outSpatialState)
{
  outSpatialState = _sensorsInput;
}

void Controls::GetCurrentSetpoint(SpatialState &outSpatialState)
{
  outSpatialState = _setPoint;
}

void Controls::SetNewMotorValues(void)
{
  int motorBRVal, motorBLVal, motorBCVal, motorFRVal, motorFLVal;

  // Init motor values at the neutral point
  motorBRVal = MOTOR_NEUTRAL;
  motorBLVal = MOTOR_NEUTRAL;
  motorBCVal = MOTOR_NEUTRAL;
  motorFRVal = MOTOR_NEUTRAL;
  motorFLVal = MOTOR_NEUTRAL;

  // Pitch correction
  motorBCVal += _PIDOutput.pitch;
  motorFLVal -= _PIDOutput.pitch/2;
  motorFRVal -= _PIDOutput.pitch/2;

  // Roll correction
  motorFLVal += _PIDOutput.roll;
  motorFRVal -= _PIDOutput.roll;

  // Yaw correction
  motorBLVal += _PIDOutput.yaw;
  motorBRVal -= _PIDOutput.yaw;

  // Add thrust to the back motors
  // This is done directly, as thust doesn't use PID
  motorBRVal += _setPoint.thrust;
  motorBLVal += _setPoint.thrust;

  // Add depth to depth motors
  motorFRVal += _PIDOutput.depth/2;
  motorFLVal += _PIDOutput.depth/2;
  motorBCVal += _PIDOutput.depth;

  // Contrain to motor limits
  motorBRVal = MOTOR_CONSTRAIN(motorBRVal);
  motorBLVal = MOTOR_CONSTRAIN(motorBLVal);
  motorBCVal = MOTOR_CONSTRAIN(motorBCVal);
  motorFRVal = MOTOR_CONSTRAIN(motorFRVal);
  motorFLVal = MOTOR_CONSTRAIN(motorFLVal);

  // Check if the props are installed in reverse
  motorBRVal = (MOTOR_BR_REVERSED) ? (MOTOR_NEUTRAL*2 - motorBRVal) : motorBRVal;
  motorBLVal = (MOTOR_BL_REVERSED) ? (MOTOR_NEUTRAL*2 - motorBLVal) : motorBLVal;
  motorBCVal = (MOTOR_BC_REVERSED) ? (MOTOR_NEUTRAL*2 - motorBCVal) : motorBCVal;
  motorFRVal = (MOTOR_FR_REVERSED) ? (MOTOR_NEUTRAL*2 - motorFRVal) : motorFRVal;
  motorFLVal = (MOTOR_FL_REVERSED) ? (MOTOR_NEUTRAL*2 - motorFLVal) : motorFLVal;

  // Write to motors
  _motorBC.write(motorBCVal);
  _motorBL.write(motorBLVal);
  _motorBR.write(motorBRVal);
  _motorFL.write(motorFLVal);
  _motorFR.write(motorFRVal);

#if PRINT_MOTOR_VALUES
  Serial.print("BC: ");
  Serial.print(motorBCVal);
  Serial.print("\tBL: ");
  Serial.print(motorBLVal);
  Serial.print("\tBR: ");
  Serial.print(motorBRVal);
  Serial.print("\tFL: ");
  Serial.print(motorFLVal);
  Serial.print("\tFR: ");
  Serial.println(motorFRVal);
#endif //PRINT_MOTOR_VALUES
}

void Controls::Start(void)
{
    // Attach motor instances to their pins
  _motorBR.attach(PORTBR);
  _motorBL.attach(PORTBL);
  _motorBC.attach(PORTBC);
  _motorFR.attach(PORTFR);
  _motorFL.attach(PORTFL);

  delay(MOTOR_STARTUP_DELAY_MS);

  _motorBR.write(MOTOR_NEUTRAL);
  _motorBL.write(MOTOR_NEUTRAL);
  _motorBC.write(MOTOR_NEUTRAL);
  _motorFR.write(MOTOR_NEUTRAL);
  _motorFL.write(MOTOR_NEUTRAL);

  // Set current yaw as 0
  _attitude.ZeroYaw();

  // Set PID control to automatic, turns them on
  _pitchPID.SetMode(AUTOMATIC);
  _rollPID.SetMode(AUTOMATIC);
  _yawPID.SetMode(AUTOMATIC);
  _depthPID.SetMode(AUTOMATIC);

}

void Controls::CalibratePressureSensor(void)
{
  _basePressure = _pSensor.getPressure(ADC_4096);
  //might consider adding value for max nextDepth
}

double Controls::GetDepth(void)
{
  double absPressure = _pSensor.getPressure(ADC_4096);
  double depth = abs(_basePressure - absPressure) * PRESSURE_TO_DEPTH_M;
  
  return depth;
}

void Controls::Stop(void)
{
  _motorBC.write(MOTOR_NEUTRAL);
  _motorBL.write(MOTOR_NEUTRAL);
  _motorBR.write(MOTOR_NEUTRAL);
  _motorFL.write(MOTOR_NEUTRAL);
  _motorFR.write(MOTOR_NEUTRAL);
}
