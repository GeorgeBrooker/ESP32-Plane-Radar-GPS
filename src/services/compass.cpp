#include "services/compass.h"

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <cmath>

#include "config.h"

namespace services::compass {

namespace {

// QMC5883L registers.
constexpr uint8_t kRegDataX = 0x00;
constexpr uint8_t kRegStatus = 0x06;
constexpr uint8_t kRegControl1 = 0x09;
constexpr uint8_t kRegControl2 = 0x0A;
constexpr uint8_t kRegSetReset = 0x0B;
constexpr uint8_t kStatusDataReadyBit = 0x01;

constexpr char kPrefsNamespace[] = "compass";
constexpr char kKeyMinX[] = "minX";
constexpr char kKeyMaxX[] = "maxX";
constexpr char kKeyMinY[] = "minY";
constexpr char kKeyMaxY[] = "maxY";

bool s_available = false;
bool s_heading_valid = false;
float s_heading_deg = 0.0f;

bool s_calibrating = false;
int16_t s_cal_min_x = 0;
int16_t s_cal_max_x = 0;
int16_t s_cal_min_y = 0;
int16_t s_cal_max_y = 0;

// Persisted hard-iron offsets (center of the min/max box for each axis).
int16_t s_offset_x = 0;
int16_t s_offset_y = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(config::kCompassI2cAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRawSample(int16_t* out_x, int16_t* out_y, int16_t* out_z) {
  Wire.beginTransmission(config::kCompassI2cAddr);
  Wire.write(kRegStatus);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<int>(config::kCompassI2cAddr), 1) != 1) {
    return false;
  }
  const uint8_t status = Wire.read();
  if ((status & kStatusDataReadyBit) == 0) {
    return false;
  }

  Wire.beginTransmission(config::kCompassI2cAddr);
  Wire.write(kRegDataX);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<int>(config::kCompassI2cAddr), 6) != 6) {
    return false;
  }
  const uint8_t xl = Wire.read();
  const uint8_t xh = Wire.read();
  const uint8_t yl = Wire.read();
  const uint8_t yh = Wire.read();
  const uint8_t zl = Wire.read();
  const uint8_t zh = Wire.read();

  *out_x = static_cast<int16_t>((xh << 8) | xl);
  *out_y = static_cast<int16_t>((yh << 8) | yl);
  *out_z = static_cast<int16_t>((zh << 8) | zl);
  return true;
}

void loadCalibration() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, true)) {
    return;
  }
  const int16_t min_x = prefs.getShort(kKeyMinX, 0);
  const int16_t max_x = prefs.getShort(kKeyMaxX, 0);
  const int16_t min_y = prefs.getShort(kKeyMinY, 0);
  const int16_t max_y = prefs.getShort(kKeyMaxY, 0);
  prefs.end();
  if (max_x > min_x && max_y > min_y) {
    s_offset_x = static_cast<int16_t>((min_x + max_x) / 2);
    s_offset_y = static_cast<int16_t>((min_y + max_y) / 2);
  }
}

void saveCalibration() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, false)) {
    return;
  }
  prefs.putShort(kKeyMinX, s_cal_min_x);
  prefs.putShort(kKeyMaxX, s_cal_max_x);
  prefs.putShort(kKeyMinY, s_cal_min_y);
  prefs.putShort(kKeyMaxY, s_cal_max_y);
  prefs.end();
  s_offset_x = static_cast<int16_t>((s_cal_min_x + s_cal_max_x) / 2);
  s_offset_y = static_cast<int16_t>((s_cal_min_y + s_cal_max_y) / 2);
}

float wrap360(float deg) {
  float v = fmodf(deg, 360.0f);
  if (v < 0.0f) {
    v += 360.0f;
  }
  return v;
}

/** Diagnostic aid: log any I2C devices found when the QMC5883L doesn't ACK. */
void scanAndLogI2cBus() {
  Serial.println("compass: QMC5883L not found at 0x0D, scanning I2C bus...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("compass: device found at 0x%02X\n", addr);
      ++found;
    }
  }
  if (found == 0) {
    Serial.println(
        "compass: no I2C devices found at all — check wiring/power (SDA/SCL/GND/3V3)");
  }
}

}  // namespace

void init() {
  Wire.begin(config::kCompassPinSda, config::kCompassPinScl);

  // Soft reset, set/reset period, then continuous 200 Hz / 8G / 512 OSR.
  writeReg(kRegControl2, 0x80);
  delay(10);
  s_available = writeReg(kRegSetReset, 0x01) && writeReg(kRegControl1, 0x1D);

  if (!s_available) {
    scanAndLogI2cBus();
  }

  loadCalibration();
}

bool available() { return s_available; }

void poll() {
  if (!s_available) {
    return;
  }

  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  if (!readRawSample(&x, &y, &z)) {
    return;
  }

  if (s_calibrating) {
    s_cal_min_x = std::min(s_cal_min_x, x);
    s_cal_max_x = std::max(s_cal_max_x, x);
    s_cal_min_y = std::min(s_cal_min_y, y);
    s_cal_max_y = std::max(s_cal_max_y, y);
  }

  const float cal_x = static_cast<float>(x - s_offset_x);
  const float cal_y = static_cast<float>(y - s_offset_y);
  // atan2(x, y): 0 = north (sensor Y axis forward), clockwise positive.
  const float raw_heading =
      wrap360(atan2f(cal_x, cal_y) * (180.0f / static_cast<float>(M_PI)) +
              config::kCompassDeclinationDeg);

  if (!s_heading_valid) {
    s_heading_deg = raw_heading;
    s_heading_valid = true;
    return;
  }

  // Shortest-path low-pass filter so it doesn't stall crossing the 0/360 wrap.
  float delta = raw_heading - s_heading_deg;
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }
  s_heading_deg = wrap360(s_heading_deg + delta * config::kCompassSmoothingAlpha);
}

float headingDeg() { return s_heading_deg; }

void calibrateStart() {
  s_calibrating = true;
  s_cal_min_x = 32767;
  s_cal_max_x = -32768;
  s_cal_min_y = 32767;
  s_cal_max_y = -32768;
}

void calibrateStop() {
  if (!s_calibrating) {
    return;
  }
  s_calibrating = false;
  saveCalibration();
}

bool isCalibrating() { return s_calibrating; }

}  // namespace services::compass
