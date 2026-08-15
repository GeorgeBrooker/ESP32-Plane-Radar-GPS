#pragma once

namespace services::location {

/** Load saved lat/lon from NVS, or use config defaults. Call once before WiFi setup. */
void init();

/** Factory defaults when nothing is stored (also used for portal field prefill). */
double lat();
double lon();

/** Parse portal strings, validate, persist to NVS, update runtime values. */
bool saveFromStrings(const char* lat_str, const char* lon_str);

/** Clear stored coordinates (e.g. with WiFi credential reset). */
void clear();

/**
 * Live GPS fix takes over lat()/lon() from the stored/manual value while
 * present; not persisted (GPS re-acquires on boot). Call every loop when
 * services::gps::hasFix() is true; call clearGpsFix() when the fix is lost.
 */
void setGpsFix(double lat, double lon);
void clearGpsFix();
bool usingGpsFix();

}  // namespace services::location
