#include "motorControl.h"

#define MOTOR_STARTUP_DELAY_MS (1000)
#define MOTOR_LIMIT_MARGIN     (20)
#define MOTOR_MIN              (0 + MOTOR_LIMIT_MARGIN)
#define MOTOR_MAX              (180 - MOTOR_LIMIT_MARGIN)
#define MOTOR_NEUTRAL          ((MOTOR_MAX + MOTOR_MIN)/2)

#define PID_SAMPLE_TIME_MS (20)
#define PID_OUTPUT_MAX     (MOTOR_MAX - MOTOR_NEUTRAL)
#define PID_OUTPUT_MIN     (MOTOR_MIN - MOTOR_NEUTRAL)

#define PID_PITCH_KP 1
#define PID_PITCH_KI 0
#define PID_PITCH_KD 0

#define PID_ROLL_KP  1
#define PID_ROLL_KI  0
#define PID_ROLL_KD  0

#define PID_YAW_KP   1
#define PID_YAW_KI   0
#define PID_YAW_KD   0

#define PRINT_MOTOR_VALUES 1

static int PORTBR = 13;
static int PORTBL = 12;
static int PORTBC = 11;
static int PORTFR = 9;
static int PORTFL = 10;

Orienter::Orienter(void) :   pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT),
  rollPID(&rollInput, &rollOutput, &rollSetpoint, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, DIRECT),
  yawPID(&yawInput, &yawOutput, &yawSetpoint, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT)
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

  // Attach motor instances to their pins
  motorBR.attach(PORTBR);
  motorBL.attach(PORTBL);
  motorBC.attach(PORTBC);
  motorFR.attach(PORTFR);
  motorFL.attach(PORTFL);

  delay(MOTOR_STARTUP_DELAY_MS);

  motorBR.write(MOTOR_NEUTRAL);
  motorBL.write(MOTOR_NEUTRAL);
  motorBC.write(MOTOR_NEUTRAL);
  motorFR.write(MOTOR_NEUTRAL);
  motorFL.write(MOTOR_NEUTRAL);
}

void Orienter::stabilize(void)
{
  // Get newest IMU data on pitch roll and yaw
  attitude.getUpdatedAxes(&pitchInput, &rollInput, &yawInput);

  // Compute new values from the PID loop (saved in the 
  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();

  // Set motors with the new PID output values
  setMotors();
}

void Orienter::setOrientation(double pitch, double roll, double yaw)
{
  pitchSetpoint = pitch;
  rollSetpoint = roll;
  yawSetpoint = yaw;
}

void Orienter::setMotors(void)
{
  int motorBRVal, motorBLVal, motorBCVal, motorFRVal, motorFLVal;

  // Init motor values at the neutral point
  motorBRVal = MOTOR_NEUTRAL;
  motorBLVal = MOTOR_NEUTRAL;
  motorBCVal = MOTOR_NEUTRAL;
  motorFRVal = MOTOR_NEUTRAL;
  motorFLVal = MOTOR_NEUTRAL;

  // Pitch correction
  motorBCVal += pitchOutput;
  motorFLVal -= pitchOutput/2;
  motorFRVal -= pitchOutput/2;

  // Roll correction  
  motorFLVal += rollOutput;
  motorFRVal -= rollOutput;

  // Yaw correction UNUSED
  motorBLVal += yawOutput;
  motorBRVal -= yawOutput;

  // Write to motors
  motorBC.write(motorBCVal);
  motorBL.write(motorBLVal);
  motorBR.write(motorBRVal);
  motorFL.write(motorFLVal);
  motorFR.write(motorFRVal);

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

