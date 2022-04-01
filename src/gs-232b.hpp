#ifndef __GS232B_HPP__
#define __GS232B_HPP__

#include <Arduino.h>

class GS232B {

public:
    GS232B();
    virtual ~GS232B() {};
    void init(HardwareSerial* serial);
    void setPosition(float azimuth, float elevation);
    void setNorthPassing(bool northPassing);

private:
    HardwareSerial* _serial;
    bool _northPassing = false;
};

#endif // __GS232B_HPP__