#include "services/compass.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_QMC5883P.h>

#include "services/gps.h"

#include "config.h"

namespace services::compass { // Begin Public Namespace

namespace { // Begin Private Namespace 
    Adafruit_QMC5883P qmc;
    bool s_available = false;
    bool s_calibrating = false;
    float s_heading_deg = 0.0f;
} // End Private namespace

// Begin Public namespace
void init() {
    Wire.begin(config::kCompassPinSda, config::kCompassPinScl);
    Wire.setClock(100000);
    Wire.setTimeout(10);
    Serial.println("Initialising QMC magnetometer chip...");
    if (!qmc.begin()) {
        Serial.println("@@ Failed to find QMC5883P chip! @@");
        return;
    }
    Serial.println(" -Found QMC5883P chip!");

    qmc.setMode(QMC5883P_MODE_NORMAL);
    qmc.setODR(QMC5883P_ODR_100HZ);
    qmc.setDSR(QMC5883P_DSR_2); // Relativly stable platform (not a fighter jet)
    qmc.setOSR(QMC5883P_OSR_4); // 2,8,2 set high to help with vibration or nearby magnetic fields
    qmc.setRange(QMC5883P_RANGE_2G); // Want to detect earth magnatic field
    qmc.setSetResetMode(QMC5883P_SETRESET_ON);
    s_available = true;
}

bool available() { return s_available; }

void poll() {
    if (qmc.isDataReady()) {
        int16_t x, y, z;
        if (qmc.getRawMagnetic(&x, &y, &z)) {
            float heading = atan2(y, x);
            
            // add magnetic declination
            float declinationDeg = services::gps::magneticDeclination();
            float declinationRad = declinationDeg * PI / 180.0;
            heading += declinationRad;

            // handle wrapping (negative degrees or degrees over 360)
            if (heading < 0) {
                heading += 2 * PI;
            } if (heading >= 2 * PI) {
                heading -= 2 * PI;
            }

            // convert to deg
            s_heading_deg = heading * 180 / PI;
        }
    }
}

float headingDeg() { return s_heading_deg; }

void calibrateStart() {
  s_calibrating = true;
}

void calibrateStop() {
  if (!s_calibrating) {
    return;
  }
  s_calibrating = false;
  // save calibration here
}

bool isCalibrating() { return s_calibrating; }

} // End public namespace
