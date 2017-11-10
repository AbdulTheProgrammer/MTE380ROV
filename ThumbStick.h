/*
 * ThumbStick.h - library for getting arduino Thumbstick readings
 * Created by Abhishek and Abdul
 * 
 */

 #ifndef ThumbStick_h
 #define ThumbStick_h

 #include "Arduino.h"

 class ThumbStick
 {
    public:
      ThumbStick(int lxPin, int lyPin, int ldPin, int rxPin, int ryPin, int rdPin);
      int readLX();
      int readLY();
      int readRX();
      int readRY();

    private:
      int _lxPin;
      int _lyPin;
      int _ldPin;
      int _rxPin;
      int _ryPin;
      int _rdPin;
      const int _deadzoneLow = 400;
      const int _deadzoneHigh = 500;
      int getMappedValue(int sensorVal);
 };

 #endif
