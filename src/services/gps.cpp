#include "services/gps.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WMM_Tinier.h>

#include "config.h"

namespace services::gps {

namespace {

HardwareSerial s_serial(1);
TinyGPSPlus s_gps;
WMM_Tinier wmm;


}  // namespace

void init() {
  s_serial.begin(config::kGpsBaud, SERIAL_8N1, config::kGpsPinRx,
                 config::kGpsPinTx);
  wmm.begin();

}

void poll() {
  while (s_serial.available() > 0) {
    s_gps.encode(s_serial.read());
  }
}

bool hasFix() {
  return s_gps.location.isValid() &&
         s_gps.location.age() < config::kGpsFixStaleMs;
}

double lat() { return s_gps.location.lat(); }

double lon() { return s_gps.location.lng(); }

float magneticDeclination() {
  if (!s_gps.date.isValid() || !s_gps.location.isValid()) {
    return 0.0; 
  } // early exit if we dont have GPS fix yet

  uint8_t day = s_gps.date.day();
  uint8_t month = s_gps.date.month();
  uint8_t year = (uint8_t)(s_gps.date.year() % 100); // cast YYYY to YY

  return wmm.magneticDeclination(lat(), lon(), year, month, day);
}

float speedKnots() { return s_gps.speed.isValid() ? s_gps.speed.knots() : 0.0f; }

float courseDeg() { return s_gps.course.isValid() ? s_gps.course.deg() : 0.0f; }

int satellites() {
  return s_gps.satellites.isValid() ? static_cast<int>(s_gps.satellites.value())
                                    : 0;
}

}  // namespace services::gps
