#pragma once

namespace services::gps {

/** Start GPS UART. Call once during setup. */
void init();

/** Feed available NMEA bytes to the parser. Call every loop iteration. */
void poll();

/** True if we have a recent fix (age < kGpsFixStaleMs). */
bool hasFix();

double lat();
double lon();

/** Ground speed/course, only meaningful when hasFix() is true. */
float speedKnots();
float courseDeg();

/** Satellites in view, for diagnostics on a status screen if desired. */
int satellites();

}  // namespace services::gps
