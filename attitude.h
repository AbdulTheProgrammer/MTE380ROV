#pragma once

#include <MPU9250.h>
#include <Kalman.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

class Attitude
{
private:
	// IMU instance
	MPU9250 imu;

	// Kalman filter instances
	Kalman kalmanX;
	Kalman kalmanY;
  Kalman kalmanZ

  // Raw axes
  double pitch_raw, roll_raw, yaw_raw;

	// Calcualted angle using  Kalman filter
	double kalAngleX, kalAngleY, kalAngleZ

	// Timer for measuring time step
	uint32_t timer;

  void updateKalmanAngles(void);
  void updateKalmanPitchRoll(void);
  void updateKalmanYaw(void);

public:
	int init(void);
	void getUpdatedAxes(double *pitch, double *roll, double *yaw);
};
