#include "gs-232b.hpp"

GS232B gs_232b = GS232B();

GS232B::GS232B() {}

void GS232B::init(HardwareSerial* serial) {
    _serial = serial;
}

void GS232B::setPosition(float azimuth, float elevation) {

    // if the azimuth passes the north pole we flip the elevation and azimuth
    if (_northPassing) {
        elevation = 180.0 - elevation;
        azimuth = azimuth + 180.0 - 360.0;
        if (azimuth < 0.0) {
            azimuth = 360.0 + azimuth;
        }
    }

    _serial->print(azimuth);
    _serial->print(",");
    _serial->println(elevation);
}

void GS232B::setNorthPassing(bool northPassing) {
    _northPassing = northPassing;
}
