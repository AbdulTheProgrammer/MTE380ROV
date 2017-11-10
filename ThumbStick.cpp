// ThumbStick.cpp file

#include "Arduino.h"
#include "ThumbStick.h"


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

ThumbStick::readLX()
{
    int reading = getMappedValue(analogRead(_lxPin));
    return reading;
}

ThumbStick::readLY()
{
    int reading = getMappedValue(analogRead(_lyPin));
    return reading;
}

ThumbStick::readRX()
{
    int reading = getMappedValue(analogRead(_rxPin));
    return reading;
}

ThumbStick::readRY()
{
    int reading = getMappedValue(analogRead(_ryPin));
    return reading;
}

ThumbStick::getMappedValue(int sensorVal)
{
    //map values from -100 to 100
    //dead zone 400 to 500
    int mappedVal = 0;
    if(sensorVal > _deadzoneHigh){
        mappedVal = map(sensorVal, _deadzoneHigh, 1023, 0, 100);
    }
    else if (sensorVal < _deadzoneLow)
    {
        mappedVal = map(sensorVal, _deadzoneLow, 0, 0, -100);
    }
    
    return mappedVal;
}

