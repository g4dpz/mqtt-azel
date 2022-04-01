#include "gs-232b.hpp"

GS232B gs_232b = GS232B();

GS232B::GS232B() {}

void GS232B::init(HardwareSerial* serial) {
    _serial = serial;
}

void GS232B::setPosition(float azimuth, float elevation) {
    _serial->print(azimuth);
    _serial->print(",");
    _serial->println(elevation);
}
