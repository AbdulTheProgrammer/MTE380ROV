#include "attitude.h"

#define KALMAN_Q_ANGLE 0.1f // Try changing to around 0.01f

Attitude::Attitude(void) : imu(NOT_SPI)
{
  // Set kalman coefficients
  kalmanX.setQangle(KALMAN_Q_ANGLE);
  kalmanY.setQangle(KALMAN_Q_ANGLE);
  kalmanZ.setQangle(KALMAN_Q_ANGLE);

  // Clear initial conditions
  isInitUpsideDown = false;
  yaw_offset = 0;
}

bool Attitude::calibrateMPU9250(void)
{
    // First test the I2C connection
    byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
//    Serial.print(F("MPU9250 I AM 0x"));
//    Serial.print(c, HEX);
//    Serial.print(F(" I should be 0x"));
//    Serial.println(0x71, HEX);
  
    if (c != 0x71) // WHO_AM_I should always be 0x71
    {
        Serial.println(F("Could not read MPU address."));
      return false;
    }
    Serial.println(F("MPU9250 is online..."));
  
   // Start by performing self test and reporting values
    imu.MPU9250SelfTest(imu.selfTest);
//    Serial.print(F("x-axis self test: acceleration trim within : "));
//    Serial.print(imu.selfTest[0],1); Serial.println("% of factory value");
//    Serial.print(F("y-axis self test: acceleration trim within : "));
//    Serial.print(imu.selfTest[1],1); Serial.println("% of factory value");
//    Serial.print(F("z-axis self test: acceleration trim within : "));
//    Serial.print(imu.selfTest[2],1); Serial.println("% of factory value");
//    Serial.print(F("x-axis self test: gyration trim within : "));
//    Serial.print(imu.selfTest[3],1); Serial.println("% of factory value");
//    Serial.print(F("y-axis self test: gyration trim within : "));
//    Serial.print(imu.selfTest[4],1); Serial.println("% of factory value");
//    Serial.print(F("z-axis self test: gyration trim within : "));
//    Serial.print(imu.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

    imu.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 init");

    // Check if the IMU is upside down
    imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values
    isInitUpsideDown = (imu.accelCount[2] < 0) ? true : false;
    
    // Get sensor resolutions, only need to do this once
    imu.getAres();
    imu.getGres();

    return true;
}

bool Attitude::calibrateAK8963(void)
{
    // First test the I2C connection
    byte d = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
//    Serial.print("AK8963 ");
//    Serial.print("I AM 0x");
//    Serial.print(d, HEX);
//    Serial.print(" I should be 0x");
//    Serial.println(0x48, HEX);
  
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
    //Serial.println("AK8963 initialized for active data mode....");

    //  Serial.println("Calibration values: ");
//    Serial.print("X-Axis factory sensitivity adjustment value ");
//    Serial.println(imu.factoryMagCalibration[0], 2);
//    Serial.print("Y-Axis factory sensitivity adjustment value ");
//    Serial.println(imu.factoryMagCalibration[1], 2);
//    Serial.print("Z-Axis factory sensitivity adjustment value ");
//    Serial.println(imu.factoryMagCalibration[2], 2);
    
    // Get sensor resolutions, only need to do this once
    imu.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    imu.magCalMPU9250(imu.magBias, imu.magScale);
//    Serial.println("AK8963 mag biases (mG)");
//    Serial.println(imu.magBias[0]);
//    Serial.println(imu.magBias[1]);
//    Serial.println(imu.magBias[2]);
//
//    Serial.println("AK8963 mag scale (mG)");
//    Serial.println(imu.magScale[0]);
//    Serial.println(imu.magScale[1]);
//    Serial.println(imu.magScale[2]);
//
//    Serial.println("Magnetometer:");
//    Serial.print("X-Axis sensitivity adjustment value ");
//    Serial.println(imu.factoryMagCalibration[0], 2);
//    Serial.print("Y-Axis sensitivity adjustment value ");
//    Serial.println(imu.factoryMagCalibration[1], 2);
//    Serial.print("Z-Axis sensitivity adjustment value ");
//    Serial.println(imu.factoryMagCalibration[2], 2);
    Serial.println("AK8963 init.");

    return true;
}

void Attitude::ZeroYaw(void)
{
    // Get initial IMU data
    updateRawOrientation();

    // Set the current yaw as the new yaw offset
    yaw_offset += rawOrientation.yaw;
}

void Attitude::getUpdatedOrientation(Orientation &inOrientation)
{
    // Read raw IMU data
    updateRawOrientation();

    //Pass raw data to filters
    updateKalmanFilters();

    // Update the output Orientation struct
    inOrientation.pitch = filteredOrientation.pitch;
    inOrientation.roll  = filteredOrientation.roll;
    inOrientation.yaw   = filteredOrientation.yaw; 

#if PRINT_ORIENTATION
    Serial.print("Pitch: ");
    Serial.print(filteredOrientation.pitch);
    Serial.print("\tRoll: ");
    Serial.print(filteredOrientation.roll);
    Serial.print("\tYaw: ");
    Serial.println(filteredOrientation.yaw);
#endif
}

void Attitude::updateKalmanFilters(void)
{

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((rawOrientation.roll < -90 && filteredOrientation.roll > 90) || (rawOrientation.roll > 90 && filteredOrientation.roll < -90)) {
        kalmanX.setAngle(rawOrientation.roll);
        filteredOrientation.roll = rawOrientation.roll;
    } else {
        filteredOrientation.roll = kalmanX.getAngle(rawOrientation.roll, gyroXrate, imu.deltat); // Calculate the angle using a Kalman filter
    }

    if (abs(filteredOrientation.roll) > 90) {
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    filteredOrientation.pitch = kalmanY.getAngle(rawOrientation.pitch, gyroYrate, imu.deltat);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((rawOrientation.pitch < -90 && filteredOrientation.pitch > 90) || (rawOrientation.pitch > 90 && filteredOrientation.pitch < -90)) {
        kalmanY.setAngle(rawOrientation.pitch);
        filteredOrientation.pitch = rawOrientation.pitch;
    } else {
        filteredOrientation.pitch = kalmanY.getAngle(rawOrientation.pitch, gyroYrate, imu.deltat); // Calculate the angle using a Kalman filter
    }

    if (abs(filteredOrientation.pitch) > 90) {
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    filteredOrientation.roll = kalmanX.getAngle(rawOrientation.roll, gyroXrate, imu.deltat); // Calculate the angle using a Kalman filter
#endif

    // This fixes the transition problem when the magnetometer angle jumps between -180 and 180 degrees
    if ((rawOrientation.yaw < -90 && filteredOrientation.yaw > 90) || (rawOrientation.yaw > 90 && filteredOrientation.yaw < -90)) {
        kalmanZ.setAngle(rawOrientation.yaw);
        filteredOrientation.yaw = rawOrientation.yaw;
    } else {
        filteredOrientation.yaw = kalmanZ.getAngle(rawOrientation.yaw, gyroZrate, imu.deltat); // Calculate the angle using a Kalman filter
    }
}

void Attitude::updateRawOrientation(void)
{
    // Get raw 3D vectors from IMU
    getRawIMUVectors();

    // Convert 3D vectors into pitch/roll/yaw
    convertVectorsToOrientation();

    // Get the offset from the initial position of the IMU
    adjustForInitialOffset();

    // Get gyro rates
    gyroXrate = imu.gx / 131.0; // Convert to deg/s
    gyroYrate = imu.gy / 131.0; // Convert to deg/s
    gyroZrate = imu.gz / 131.0; // Convert to deg/s
}

void Attitude::getRawIMUVectors(void)
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
    // Note that we have to get the magnetometer on the same coordinates as the accel/gyro,
    // By changing the axes to be like:
    //    newMagX  = imu.my;
    //    newMagY  = imu.mx;
    //    newMagZ  = -imu.mz;
    
    imu.my = (float)imu.magCount[0] * imu.mRes
               * imu.factoryMagCalibration[0] - imu.magBias[0];
    imu.mx = (float)imu.magCount[1] * imu.mRes
               * imu.factoryMagCalibration[1] - imu.magBias[1];
    imu.mz = -((float)imu.magCount[2] * imu.mRes
               * imu.factoryMagCalibration[2] - imu.magBias[2]);

    // Get time interval from last call
    imu.updateTime();

  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01
  else {
      Serial.println("IMU data not ready; sample rate needs to be increased.!");
      delay(5000);
  }
}

void Attitude::convertVectorsToOrientation(void)
{
      // Convert raw vectors into raw pitch roll and yaw
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26

    rawOrientation.roll  = atan2(imu.ay, imu.az);
    rawOrientation.pitch = atan(-imu.ax / sqrt(imu.ay * imu.ay + imu.az * imu.az));
#else // Eq. 28 and 29
    rawOrientation.roll  = atan(imu.ay / sqrt(imu.ax * imu.ax + imu.az * imu.az));
    rawOrientation.pitch = atan2(-imu.ax, imu.az);
#endif

    double Bfy = imu.mz * sin(rawOrientation.roll) - imu.my * cos(rawOrientation.roll);
    double Bfx = imu.mx * cos(rawOrientation.pitch) + imu.my * sin(rawOrientation.pitch) * sin(rawOrientation.roll) + imu.mz * sin(rawOrientation.pitch) * cos(rawOrientation.roll);
    rawOrientation.yaw = atan2(-Bfy, Bfx);

    // Convert pitch roll yaw to degrees
    rawOrientation.pitch = rawOrientation.pitch * RAD_TO_DEG;
    rawOrientation.roll = rawOrientation.roll * RAD_TO_DEG;
    rawOrientation.yaw = rawOrientation.yaw * RAD_TO_DEG;
}

void Attitude::adjustForInitialOffset(void)
{
  // If the IMU is upside down, flip the roll values
    if (isInitUpsideDown)
    {
      rawOrientation.roll = (rawOrientation.roll > 0) ? rawOrientation.roll - 180: 180 + rawOrientation.roll;
    }
    // Map yaw to the offset
    rawOrientation.yaw = rawOrientation.yaw - yaw_offset;
    if (rawOrientation.yaw > 180)
    {
      rawOrientation.yaw -= 360;
    }
    else if (rawOrientation.yaw < -180)
    {
      rawOrientation.yaw += 360;
    }
}

