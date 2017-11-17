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
    pinMode(rdPin, INPUT);
    pinMode(ldPin, INPUT);

    _lxPin = lxPin;
    _lyPin = lyPin;
    _ldPin = ldPin;
    
    _rxPin = rxPin;
    _ryPin = ryPin;
    _rdPin = rdPin;
}

int ThumbStick::readLX()
{
    int reading = getMappedValue(analogRead(_lxPin));
    return reading;
}

int ThumbStick::readLY()
{
    int reading = getMappedValue(analogRead(_lyPin));
    return reading;
}

int ThumbStick::readRX()
{
    int reading = getMappedValue(analogRead(_rxPin));
    return reading;
}

int ThumbStick::readRY()
{
    int reading = getMappedValue(analogRead(_ryPin));
    return reading;
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

