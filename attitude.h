#pragma once

#include <MPU9250.h>
#include <Kalman.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

typedef struct Orientation {
  double pitch;
  double roll;
  double yaw;
} Orientation;

class Attitude
{
private:
	// IMU instance
	MPU9250 imu;

	// Kalman filter instances
	Kalman kalmanX;
	Kalman kalmanY;
  Kalman kalmanZ;

  // Raw orientation and raw gyro values 
  Orientation rawOrientation;
  double gyroXrate, gyroYrate, gyroZrate;
 
  // Filtered orientation after Kalman filter
  Orientation filteredOrientation;

  // Initial position and yaw offset
  bool isInitUpsideDown;
  double yaw_offset;

	// Timer for measuring time step
	uint32_t timer;

  void updateRawOrientation(void);
  void getRawIMUVectors(void);
  void convertVectorsToOrientation(void);
  void adjustForInitialOffset(void);
  void updateKalmanFilters(void);

public:
	Attitude(void);

    /*!
   * \brief   Starts the MPU9250, and calibrates the IMU. 
   * 
   * \details This should be called when the IMU is completely upright on a flat surface.
   *          This process should take about 5 seconds.
   */
  bool calibrateMPU9250(void);
  
  /*!
   * \brief   Starts the AK8963, and calibrates the IMU. 
   * 
   * \details Following this call will be a calibration for 15 seconds.
   *          The IMU shuold be waved in a figure 8 during this time.
   *          After 15 seconds is over, the IMU yaw position at this time 
   *          is calibrated as the 0-yaw position.
   */
  bool calibrateAK8963(void);

  /*!
   * \brief   Gets the current (filtered) orientation of the IMU.
   */
	void getUpdatedOrientation(Orientation &inOrientation);
};
