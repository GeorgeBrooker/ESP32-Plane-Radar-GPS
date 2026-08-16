#include "services/compass.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_QMC5883P.h>
#include <Preferences.h>
#include <cmath>

#include "services/gps.h"

#include "config.h"

namespace services::compass { // Begin Public Namespace

namespace { // Begin Private Namespace 
    Adafruit_QMC5883P qmc;
    bool s_available = false;
    bool s_calibrating = false;
    float s_heading_deg = 0.0f;
    
    // Hard-iron calibration: offsets from min/max tracking during rotation
    float s_cal_x_offset = 0.0f;
    float s_cal_y_offset = 0.0f;
    int16_t s_cal_x_min = 0, s_cal_x_max = 0;
    int16_t s_cal_y_min = 0, s_cal_y_max = 0;

    float clampCompass(float heading) {
        // 1. Get the remainder within the -360.0 to 360.0 range
        float wrapped = std::fmod(heading, 360.0f);
        
        // 2. If the result is negative, add 360 to bring it into the positive domain
        if (wrapped < 0.0f) {
            wrapped += 360.0f;
        }
        
        return wrapped;
    }
    float invertCompassDirection(float heading) {
        return 360.0f - heading;
    }
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
    qmc.setDSR(QMC5883P_DSR_8); // Relativly stable platform (not a fighter jet)
    qmc.setOSR(QMC5883P_OSR_8); // 2,8,2 set high to help with vibration or nearby magnetic fields
    qmc.setRange(QMC5883P_RANGE_8G); // Sensitivity essentially
    qmc.setSetResetMode(QMC5883P_SETRESET_ON);
    s_available = true;    
    // Load calibration from NVS if previously saved
    Preferences prefs;
    if (prefs.begin("compass", true)) {
        s_cal_x_offset = prefs.getFloat("cal_x", 0.0f);
        s_cal_y_offset = prefs.getFloat("cal_y", 0.0f);
        prefs.end();
        if (s_cal_x_offset != 0.0f || s_cal_y_offset != 0.0f) {
            Serial.printf("Compass calibration loaded: x_off=%.1f y_off=%.1f\n", 
                          s_cal_x_offset, s_cal_y_offset);
        }
    }
}

bool available() { return s_available; }

void poll() {
    if (qmc.isDataReady()) {
        int16_t x, y, z;
        if (qmc.getRawMagnetic(&x, &y, &z)) {
            // Track min/max if calibrating
            if (s_calibrating) {
                if (x < s_cal_x_min) s_cal_x_min = x;
                if (x > s_cal_x_max) s_cal_x_max = x;
                if (y < s_cal_y_min) s_cal_y_min = y;
                if (y > s_cal_y_max) s_cal_y_max = y;
            }
            
            // Apply hard-iron offset
            float cal_x = x - s_cal_x_offset;
            float cal_y = y - s_cal_y_offset;
            float heading = atan2(cal_y, cal_x);
            
            // add magnetic declination
            heading = heading * 180 / PI; // convert to deg
            heading = heading + services::gps::magneticDeclination();

            // apply rotation offset
            heading = heading - 90.0f;
            
            //invert (based on compass placement in device)
            heading = invertCompassDirection(heading);
            
            // clamp final result to 360
            s_heading_deg = clampCompass(heading);
        }
    }
}

float headingDeg() { return s_heading_deg; }

void calibrateStart() {
  s_cal_x_min = 32767;
  s_cal_x_max = -32768;
  s_cal_y_min = 32767;
  s_cal_y_max = -32768;
  s_calibrating = true;
  Serial.println("Compass calibration started — rotate device through 360 degrees");
}

void calibrateStop() {
  if (!s_calibrating) {
    return;
  }
  s_calibrating = false;
  
  // Calculate hard-iron offsets from min/max
  s_cal_x_offset = (s_cal_x_max + s_cal_x_min) / 2.0f;
  s_cal_y_offset = (s_cal_y_max + s_cal_y_min) / 2.0f;
  
  Serial.printf("Calibration complete: x_off=%.1f y_off=%.1f\n", 
                s_cal_x_offset, s_cal_y_offset);
  
  // Save to NVS
  Preferences prefs;
  if (prefs.begin("compass", false)) {
    prefs.putFloat("cal_x", s_cal_x_offset);
    prefs.putFloat("cal_y", s_cal_y_offset);
    prefs.end();
    Serial.println("Calibration saved to NVS");
  }
}

bool isCalibrating() { return s_calibrating; }

} // End public namespace
