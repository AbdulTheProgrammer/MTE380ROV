#pragma once

#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include "attitude.h"


#define PRINT_MOTOR_VALUES 1 // Set to 1 to print values sent to motor. Set to 0 to turn off.

typedef struct SpatialState
{
  double pitch;
  double roll;
  double yaw;
  double thrust;
  double depth;
} SpatialState;

class Controls
{
 private:
  // PID controllers
  PID _pitchPID;
  PID _rollPID;
  PID _yawPID;
  PID _depthPID;

  SpatialState _setPoint, _sensorsInput, _PIDOutput; // xr, x, u

  // IMU instance
  Attitude _attitude;
  bool _isAccelGyroStarted;
  bool _isMagStarted;

  //pressure instances
  MS5803 _pSensor;
  double _basePressure;
  double _currentPressure;

  // Motor instances
  Servo _motorBL;
  Servo _motorBR;
  Servo _motorBC;
  Servo _motorFL;
  Servo _motorFR;

  void SetNewMotorValues(void);
  void CalculatePIDs(bool inStabilizePitch, bool inStabilizeRoll, bool inStabilizeYaw, bool inStabilizeDepth);
  void GetSensorsInput(void);
  //TODO need to tweak this TEMPORARY
  inline double PressureToDepth(double absPressure) {  return 0.1222*abs(_basePressure - absPressure); }
  double getUpdatedDepthChange();
  
 public:
  Controls(void);

  /*!
   * \brief   Starts and calibrates the accelerometer and gyro.
   *
   * \details This should be called when the IMU is completely upright on a flat surface.
   *          This process should take about 5 seconds.
   */
  void CalibrateAccelGyro(void);

    /*!
   * \brief   Starts and calibrates the magnetometer.
   *
   * \details Following this call will be a calibration for 15 seconds.
   *          The magnetometer shuold be waved in a figure 8 during this time.
   *          After 15 seconds is over, the IMU yaw position at this time
   *          is calibrated as the 0-yaw position.
   */
  void CalibrateMagnetometer(void);

  /*!
   * \brief   Sets the desired orientation for the IMU.
   *
   * \param   inSpatialState - Struct containing desired spatial state. Will be copied internally.
   */
  void SetDesiredSpatialState(SpatialState &inSpatialState);

  /*!
   * \brief   Gets the current orientation of the IMU.
   *
   * \param   outSpatialState - Struct to hold current spatial state.
   *                            The struct must be allocated prior to calling this function.
   */
  void GetCurrentSpatialState(SpatialState &outSpatialState);

    /*!
   * \brief   Gets the current orientation setpoint of the IMU.
   *
   * \param   outSpatialState - Struct to hold current spatial state setpoint.
   *                            The struct must be allocated prior to calling this function.
   */
  void GetCurrentSetpoint(SpatialState &outSpatialState);

  /*
   * \brief   Starts all controls. Stabilize() chould be called often after this to continuously stabilize the ROV.
   */
  void Start();

  /*!
   * \brief   Function that performs the control loop and stabilizes the system.
   *
   * \details This should be called as often as possible to produce more stable results.
   *
   * \param   stabilizePitch - Set to true if the controller should stabilize pitch.
   * \param   stabilizeRoll  - Set to true if the controller should stabilize roll.
   * \param   stabilizeYaw   - Set to true if the controller should stabilize yaw.
   * \param   stabilizeDepth - Set to true if the controller should stabilize depth.
   */
  void Stabilize(bool stabilizePitch, bool stabilizeRoll, bool stabilizeYaw, bool stabilizeDepth);

  /*
  * Calibrates pressure sensor with baseline pressure (representing top surface of water)
  */
  void calibratePressureSensor();
  /*
   * \brief   Stops all controls. Should only be called in emergency stops or when shutting down.
   */
  void Stop();
};
