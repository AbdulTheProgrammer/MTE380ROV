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

	// IMU raw data
	double accX, accY, accZ;
	double gyroX, gyroY, gyroZ;
	double magX, magY, magZ;

	// Calcualted angle using  Kalman filter
	double kalAngleX, kalAngleY;

	// Timer for measuring time step
	uint32_t timer;

  void updateKalmanAngles();

public:
	void init(void);
	void getUpdatedAxes(double *pitch, double *roll, double *yaw);
};
