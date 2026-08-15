#pragma once

namespace services::compass {

/** Start I2C and configure the QMC5883L. Call once during setup. */
void init();

/** True if the sensor ACKed on the bus during init(). */
bool available();

/** Read a sample and update the smoothed heading. Call periodically (see
 * kCompassPollIntervalMs). No-op if available() is false. */
void poll();

/** Smoothed compass heading in degrees, 0..360 (0 = north, clockwise). */
float headingDeg();

/**
 * Simple running-min/max hard-iron calibration. Rotate the device through a
 * full 360 degrees (flat) after calling start(); call stop() to persist the
 * result to NVS. Auto-loaded on init() if previously saved.
 */
void calibrateStart();
void calibrateStop();
bool isCalibrating();

}  // namespace services::compass
