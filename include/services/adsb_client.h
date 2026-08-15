#pragma once

#include <cstddef>

namespace services::adsb {

struct Aircraft {
  float lat;
  float lon;
  float nose_deg;
  float track_deg;
  float gs_knots;
  char callsign[9];
  char type[5];
  char alt[12];
};

class AircraftReader {
  public:
    AircraftReader();
    ~AircraftReader();
    AircraftReader(const AircraftReader&) = delete;
    AircraftReader& operator=(const AircraftReader&) = delete;

    const Aircraft* list() const;
    size_t count() const;
};

void init();
void startWorker();
bool consumeUpdate();

constexpr size_t kMaxAircraft = 64;

/** Hook invoked during long HTTP I/O (e.g. wifiLoop). Optional. */
using PollFn = void (*)();
void setPollFn(PollFn fn);

/** Fetch aircraft within fetch_radius_km of center_lat/lon from adsb.fi. */
bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km);

}  // namespace services::adsb
