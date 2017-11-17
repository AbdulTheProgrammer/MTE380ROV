// ThumbStick.cpp file

#include "Arduino.h"
#include "ThumbStick.h"

#define THUMBSTICK_RANGE         (1023)
#define THUMBSTICK_NEUTRAL_VALUE (THUMBSTICK_RANGE/2)

#define DEAD_ZONE_SIZE (30)
#define DEAD_ZONE_HIGH (THUMBSTICK_NEUTRAL_VALUE + DEAD_ZONE_SIZE/2)
#define DEAD_ZONE_LOW  (THUMBSTICK_NEUTRAL_VALUE - DEAD_ZONE_SIZE/2)

ThumbStick::ThumbStick(int lxPin, int lyPin, int ldPin, int rxPin, int ryPin, int rdPin)
{
    pinMode(rdPin, INPUT_PULLUP);
    pinMode(ldPin, INPUT_PULLUP);

    _lxPin = lxPin;
    _lyPin = lyPin;
    _ldPin = ldPin;
    
    _rxPin = rxPin;
    _ryPin = ryPin;
    _rdPin = rdPin;
}

int ThumbStick::readLX()
{
    return getMappedValue(analogRead(_lxPin));
}

int ThumbStick::readLY()
{
    return getMappedValue(analogRead(_lyPin));
}

int ThumbStick::readRX()
{
    return getMappedValue(analogRead(_rxPin));
}

int ThumbStick::readRY()
{
    return getMappedValue(analogRead(_ryPin));
}

bool ThumbStick::readLD()
{
  // Active low
  return !digitalRead(_ldPin);
}

bool ThumbStick::readRD()
{
  // Active low
  return !digitalRead(_rdPin);
}

int ThumbStick::getMappedValue(int sensorVal)
{
    //map values from -100 to 100
    int mappedVal = 0;
    if(sensorVal > DEAD_ZONE_HIGH){
        mappedVal = map(sensorVal, DEAD_ZONE_HIGH, 1023, 0, 100);
    }
    else if (sensorVal < DEAD_ZONE_LOW)
    {
        mappedVal = map(sensorVal, DEAD_ZONE_LOW, 0, 0, -100);
    }
    
    return mappedVal;
}

