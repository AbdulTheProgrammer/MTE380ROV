#include "attitude.h"

void Attitude::init(void)
{
    const int accelFSR = 2;
    const int gyroFSR  = 2000;
    const int accelGyroSampleRate = 100;
    const int compassSampleRate   = 100;
    const int LPF_Freq = 5;

    // Call imu.begin() to verify communication with and
    // initialize the MPU-9250 to it's default values.
    // Most functions return an error code - INV_SUCCESS (0)
    // indicates the IMU was present and successfully set up
    if (imu.begin() != INV_SUCCESS)
    {
        while (1)
        {
            Serial.println("Unable to communicate with MPU-9250");
            Serial.println("Check connections, and try again.");
            Serial.println();
            delay(3000);
        }
    }

    // Use setSensors to turn on or off MPU-9250 sensors.
    // Any of the following defines can be combined:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
    // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Enable all sensors:
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    imu.setGyroFSR(gyroFSR);
    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(accelFSR);
    // Note: the MPU-9250's magnetometer FSR is set at 
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    imu.setLPF(LPF_Freq); // Set LPF corner frequency to 5Hz

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    imu.setSampleRate(accelGyroSampleRate);

    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    imu.setCompassSampleRate(compassSampleRate);
}

void Attitude::getUpdatedAxes(double *pitch, double *roll, double *yaw)
{
    // dataReady() checks to see if new accel/gyro data
    // is available. It will return a boolean true or false
    // (New magnetometer data cannot be checked, as the library
    //  runs that sensor in single-conversion mode.)
    if ( imu.dataReady() )
    {
        // Call update() to update the imu objects sensor data.
        // You can specify which sensors to update by combining
        // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
        // UPDATE_TEMPERATURE.
        // (The update function defaults to accel, gyro, compass,
        //  so you don't have to specify these values.)
        imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

        // Pass values through the Kalman filter
        updateKalmanAngles();
    }
    else {
        Serial.println("IMU data not ready; sample rate needs to be increased.!");
        delay(5000);
    }

    *pitch = kalAngleY;
    *roll  = kalAngleX;
    *yaw   = 0; // TODO 
}

void Attitude::updateKalmanAngles(void)
{
    double dt;
    double gyroXrate;
    double gyroYrate;

    dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

     // Take raw IMU readings (previously updated)
    accX = imu.ax;
    accY = imu.ay;
    accZ = imu.az;
    gyroX = imu.gx;
    gyroY = imu.gy;
    gyroZ = imu.gz;

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    // Check if the IMU gave invalid data, resulting in NaN after atan.
    if (pitch != pitch)
    {
        Serial.println("!!!Invalid data!!!");
        Serial.print("accX ");
        Serial.print(accX);
        Serial.print(" accY ");
        Serial.print(accY);
        Serial.print(" accZ ");
        Serial.println(accZ);
        delay(3000);
    }

    gyroXrate = gyroX / 131.0; // Convert to deg/s
    gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        kalAngleX = roll;
    } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90) {
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        kalAngleY = pitch;
    } else {
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
    }

    if (abs(kalAngleY) > 90) {
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
}
