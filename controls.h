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

class Controls
{
 private: 
  // PID controllers
  PID pitchPID;
  PID rollPID;
  PID yawPID;
  
  Orientation setPoint, input, output; // xr, x, u

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
  Controls(void);

  /*!
   * \brief   Initializes all motors.
   */
  void InitializeMotors(void);

  /*!
   * \brief   Starts the MPU9250, and calibrates the IMU. 
   * 
   * \details This should be called when the IMU is completely upright on a flat surface.
   */
  void CalibateMPU9250(void);

    /*!
   * \brief   Starts the AK8963, and calibrates the IMU. 
   * 
   * \details Following this call will be a calibration for 15 seconds.
   *          The IMU shuold be waved in a figure 8 during this time.
   *          After 15 seconds is over, the IMU yaw position at this time 
   *          is calibrated as the 0-yaw position.
   */
  void CalibateAK8963(void);

  /*!
   * \brief   Sets the desired orientation for the IMU. 
   */
  void SetDesiredOrientation(const Orientation &inOrientation);
  
  /*!
   * \brief   Gets the current orientation of the IMU. (Not many uses for this)
   */
  void GetCurrentOrientation(Orientation &outOrientation);
   
  /*!
   * \brief   Function that performs the control loop and stabilizes the system. 
   * 
   * \details This should be called as often as possible to produce more stable results.
   */
  void Stabilize(void);
};
