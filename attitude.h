#pragma once

#include <Kalman.h>
#include <SparkFunMPU9250-DMP.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

class Attitude
{
private:
	// IMU instance
	MPU9250_DMP imu;

	// Kalman filter instances
	Kalman kalmanX;
	Kalman kalmanY;
  Kalman kalmanZ;

	// Calcualted angle using  Kalman filter
	double kalAngleX, kalAngleY, kalAngleZ;

	// Timer for measuring time step
	uint32_t timer;

  void updateKalmanAngles(void);
  void updateKalmanPitchRoll(double dt);
  void updateKalmanYaw(double dt);

public:
	void init(void);
	void getUpdatedAxes(double *pitch, double *roll, double *yaw);
};
