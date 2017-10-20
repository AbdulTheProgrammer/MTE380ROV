#include "attitude.h"

void Attitude::init(void)
{
    const int accelFSR = 2;
    const int gyroFSR  = 2000;
    const int accelGyroSampleRate = 200;
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
            delay(1000);
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
    double dt;

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

        
        dt = (double)(micros() - timer) / 1000000; // Calculate delta time
        timer = micros();

        // Pass values through the Kalman filter
        updateKalmanPitchRoll(dt);
        updateKalmanYaw(dt);
    }
    else {
        Serial.println("IMU data not ready; sample rate needs to be increased.!");
        delay(5000);
    }

    *pitch = kalAngleY;
    *roll  = kalAngleX;
    *yaw   = kalAngleZ; 
}

void Attitude::updateKalmanPitchRoll(double dt)
{
    double gyroXrate, gyroYrate;
    double accX, accY, accZ, gyroX, gyroY;

     // Take raw IMU readings (previously updated)
    accX  = imu.ax;
    accY  = imu.ay;
    accZ  = imu.az;
    gyroX = imu.gx;
    gyroY = imu.gy;
    
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ);
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ));
#else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ));
    double pitch = atan2(-accX, accZ);
#endif

    roll  = roll * RAD_TO_DEG;
    pitch = pitch * RAD_TO_DEG;

    gyroXrate = gyroX / 131.0; // Convert to deg/s
    gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        kalAngleX = roll;
    } else {
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    }

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

void Attitude::updateKalmanYaw(double dt)
{
    double gyroZ, gyroZrate;
    double magX, magY, magZ;
    
    magX  = imu.calcMag(imu.mx);
    magY  = imu.calcMag(imu.my);
    magZ  = imu.calcMag(imu.mz);

    Serial.print(magX);
    Serial.print("\t");
    Serial.print(magY);
    Serial.print("\t");
    Serial.println(magZ);

    gyroZrate = gyroZ / 131.0; // Convert to deg/s

    double rollAngle = kalAngleX * DEG_TO_RAD;
    double pitchAngle = kalAngleY * DEG_TO_RAD;
  
    double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
    double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
    double yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

    // This fixes the transition problem when the magnetometer angle jumps between -180 and 180 degrees
    if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
        kalmanZ.setAngle(yaw);
        kalAngleZ = yaw;
    } else {
        kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter
    }
}

