#include "attitude.h"

#define KALMAN_Q_ANGLE 0.1f

#define DO_FULL_INIT_SEQUENCE 1

int Attitude::init(void)
{
    const int accelFSR = 2;
    const int gyroFSR  = 2000;
    const int accelGyroSampleRate = 200;
    const int compassSampleRate   = 100;
    const int LPF_Freq = 5;

    byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c != 0x71) // WHO_AM_I should always be 0x71
  {
      Serial.println(F("Could not read MPU address."));
    return false;
  }
    Serial.println(F("MPU9250 is online..."));

#if DO_FULL_INIT_SEQUENCE

    // Start by performing self test and reporting values
    imu.MPU9250SelfTest(imu.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(imu.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(imu.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(imu.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(imu.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(imu.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(imu.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
    
#endif // DO_FULL_INIT_SEQUENCE

    imu.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Could not read magnetometer address."));
      Serial.flush();
      return false;
    }

    // Get magnetometer calibration from AK8963 ROM
    imu.initAK8963(imu.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    //  Serial.println("Calibration values: ");
    Serial.print("X-Axis factory sensitivity adjustment value ");
    Serial.println(imu.factoryMagCalibration[0], 2);
    Serial.print("Y-Axis factory sensitivity adjustment value ");
    Serial.println(imu.factoryMagCalibration[1], 2);
    Serial.print("Z-Axis factory sensitivity adjustment value ");
    Serial.println(imu.factoryMagCalibration[2], 2);
    
    // Get sensor resolutions, only need to do this once
    imu.getAres();
    imu.getGres();
    imu.getMres();

#if DO_FULL_INIT_SEQUENCE
    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    imu.magCalMPU9250(imu.magBias, imu.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(imu.magBias[0]);
    Serial.println(imu.magBias[1]);
    Serial.println(imu.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(imu.magScale[0]);
    Serial.println(imu.magScale[1]);
    Serial.println(imu.magScale[2]);

    Serial.println("Magnetometer:");
    Serial.print("X-Axis sensitivity adjustment value ");
    Serial.println(imu.factoryMagCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value ");
    Serial.println(imu.factoryMagCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value ");
    Serial.println(imu.factoryMagCalibration[2], 2);

    delay(2000);

#endif //DO_FULL_INIT_SEQUENCE

    kalmanX.setQangle(KALMAN_Q_ANGLE);
    kalmanY.setQangle(KALMAN_Q_ANGLE);
    kalmanZ.setQangle(KALMAN_Q_ANGLE);

//    // Set initial pitch/roll/yaw values
//    imu.updateTime();
//    delay(10);
//    imu.updateTime();
//    updateKalmanPitchRoll();
//    updateKalmanYaw();

    return true;
}

void Attitude::getUpdatedAxes(double *pitch, double *roll, double *yaw)
{
    // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    imu.ax = (float)imu.accelCount[0] * imu.aRes; // - imu.accelBias[0];
    imu.ay = (float)imu.accelCount[1] * imu.aRes; // - imu.accelBias[1];
    imu.az = (float)imu.accelCount[2] * imu.aRes; // - imu.accelBias[2];

    imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    imu.gx = (float)imu.gyroCount[0] * imu.gRes;
    imu.gy = (float)imu.gyroCount[1] * imu.gRes;
    imu.gz = (float)imu.gyroCount[2] * imu.gRes;

    imu.readMagData(imu.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    imu.mx = (float)imu.magCount[0] * imu.mRes
               * imu.factoryMagCalibration[0] - imu.magBias[0];
    imu.my = (float)imu.magCount[1] * imu.mRes
               * imu.factoryMagCalibration[1] - imu.magBias[1];
    imu.mz = (float)imu.magCount[2] * imu.mRes
               * imu.factoryMagCalibration[2] - imu.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01
    else {
        Serial.println("IMU data not ready; sample rate needs to be increased.!");
        delay(5000);
    }
    
    // Must be called before updating quaternions!
    imu.updateTime();
    updateKalmanPitchRoll();
    updateKalmanYaw();

    *pitch = kalAngleY;
    *roll  = kalAngleX;
    *yaw   = kalAngleZ; 
}

void Attitude::updateKalmanPitchRoll(void)
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
    roll_raw  = atan2(accY, accZ);
    pitch_raw = atan(-accX / sqrt(accY * accY + accZ * accZ));
#else // Eq. 28 and 29
    roll_raw  = atan(accY / sqrt(accX * accX + accZ * accZ));
    pitch_raw = atan2(-accX, accZ);
#endif

    // Invert roll since upside down IMU
    double roll  = ( roll_raw > 0 ) ? roll_raw * RAD_TO_DEG -180 : 180 + roll_raw * RAD_TO_DEG;
    double pitch = pitch_raw * RAD_TO_DEG;

    gyroXrate = gyroX / 131.0; // Convert to deg/s
    gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        kalAngleX = roll;
    } else {
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, imu.deltat); // Calculate the angle using a Kalman filter
    }

    if (abs(kalAngleX) > 90) {
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, imu.deltat);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        kalAngleY = pitch;
    } else {
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, imu.deltat); // Calculate the angle using a Kalman filter
    }

    if (abs(kalAngleY) > 90) {
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, imu.deltat); // Calculate the angle using a Kalman filter
#endif
}

void Attitude::updateKalmanYaw(void)
{
    double gyroZ, gyroZrate;
    double magX, magY, magZ;
    
    magX  = imu.my;
    magY  = imu.mx;
    magZ  = -imu.mz;

    gyroZrate = gyroZ / 131.0; // Convert to deg/s
  
    double Bfy = magZ * sin(roll_raw) - magY * cos(roll_raw);
    double Bfx = magX * cos(pitch_raw) + magY * sin(pitch_raw) * sin(roll_raw) + magZ * sin(pitch_raw) * cos(roll_raw);
    yaw_raw = atan2(-Bfy, Bfx);
    double yaw = yaw_raw * RAD_TO_DEG;
    // This fixes the transition problem when the magnetometer angle jumps between -180 and 180 degrees
    if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
        kalmanZ.setAngle(yaw);
        kalAngleZ = yaw;
    } else {
        kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, imu.deltat); // Calculate the angle using a Kalman filter
    }
}

