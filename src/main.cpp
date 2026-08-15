/**
 * Plane Radar — WiFi setup, then radar UI on the round GC9A01 display.
 */

#include <Arduino.h>
#include <WiFi.h>

#include <cmath>

#include "config.h"
#include "hardware/display.h"
#include "services/adsb_client.h"
#include "services/compass.h"
#include "services/gps.h"
#include "services/radar_location.h"
#include "services/wifi_setup.h"
#include "ui/radar_display.h"
#include "ui/radar_range.h"
#include "ui/status_screens.h"

namespace {

bool g_radar_visible = false;
unsigned long g_wifi_down_since = 0;
unsigned long g_last_reconnect_ms = 0;
unsigned long g_last_adsb_fetch_ms = 0;
unsigned long g_last_gps_poll_ms = 0;
unsigned long g_last_compass_poll_ms = 0;
unsigned long g_last_sensor_log_ms = 0;
constexpr unsigned long kSensorLogIntervalMs = 2000;
/** Redraw between ADS-B polls when GPS/heading moves enough to matter. */
constexpr double kGpsRedrawThresholdDeg = 0.00005;  // ~5 m
constexpr float kHeadingRedrawThresholdDeg = 1.0f;
double g_last_drawn_lat = 0.0;
double g_last_drawn_lon = 0.0;
float g_last_drawn_heading = 0.0f;

void showRadarIfConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    g_radar_visible = false;
    return;
  }
  ui::radarDisplayDraw();
  g_radar_visible = true;
}

void onRangeTap() {
  ui::radar::rangeNext();
  char range_label[12];
  ui::radar::formatCurrentRing3Label(range_label, sizeof(range_label));
  Serial.printf("Range: %s (outer ~%.0f km)\n", range_label,
                ui::radar::rangeCurrent().outer_km);

  if (g_radar_visible && WiFi.status() == WL_CONNECTED) {
    ui::radarDisplayDraw();
  }
}

void handleBootButton() {
  bootButtonPollLongPress();
  if (bootButtonConsumeTap()) {
    onRangeTap();
  }
}

/** Feed GPS/compass readings and report whether position/heading moved enough
 * to warrant an off-cycle redraw. */
bool pollLocationSensors() {
  const unsigned long now = millis();

  if (now - g_last_gps_poll_ms >= config::kGpsPollIntervalMs) {
    g_last_gps_poll_ms = now;
    services::gps::poll();
    if (services::gps::hasFix()) {
      services::location::setGpsFix(services::gps::lat(), services::gps::lon());
    } else {
      services::location::clearGpsFix();
    }
  }

  if (now - g_last_compass_poll_ms >= config::kCompassPollIntervalMs) {
    g_last_compass_poll_ms = now;
    services::compass::poll();
  }

  if (now - g_last_sensor_log_ms >= kSensorLogIntervalMs) {
    g_last_sensor_log_ms = now;
    Serial.printf("gps: %s fix=%d sats=%d lat=%.6f lon=%.6f | compass: %s heading=%.1f\n",
                  services::gps::hasFix() ? "fix" : "no-fix",
                  services::gps::hasFix(), services::gps::satellites(),
                  services::location::lat(), services::location::lon(),
                  services::compass::available() ? "ok" : "not found",
                  services::compass::headingDeg());
  }

  const double lat = services::location::lat();
  const double lon = services::location::lon();
  const float heading = ui::radar::rotationHeadingDeg();
  const bool moved = fabs(lat - g_last_drawn_lat) > kGpsRedrawThresholdDeg ||
                     fabs(lon - g_last_drawn_lon) > kGpsRedrawThresholdDeg ||
                     fabs(heading - g_last_drawn_heading) > kHeadingRedrawThresholdDeg;
  if (moved) {
    g_last_drawn_lat = lat;
    g_last_drawn_lon = lon;
    g_last_drawn_heading = heading;
  }
  return moved;
}

void fetchAndDrawAircraft() {
  const float fetch_km = ui::radar::fetchRadiusKm();
  if (!services::adsb::fetchUpdate(services::location::lat(),
                                   services::location::lon(), fetch_km)) {
    handleBootButton();
    return;
  }
  ui::radarDisplayRefreshAircraft();
  handleBootButton();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("Plane Radar");

  bootButtonInit();
  displayInit();
  if (wifiShowsSetupScreenOnBoot()) {
    statusScreenPortal();
  }
  services::location::init();
  services::gps::init();
  services::compass::init();
  ui::radar::rangeInit();
  services::adsb::init();
  services::adsb::startWorker();

  if (wifiSetupConnect()) {
    showRadarIfConnected();
  }
}

void loop() {
  handleBootButton();
  wifiLoop();
  const bool location_moved = pollLocationSensors();

  if (WiFi.status() != WL_CONNECTED) {
    if (g_radar_visible) {
      Serial.println("WiFi lost — will reconnect");
      g_radar_visible = false;
    }

    if (g_wifi_down_since == 0) {
      g_wifi_down_since = millis();
    }

    const unsigned long down_ms = millis() - g_wifi_down_since;
    if (down_ms >= config::kWifiDownGraceMs &&
        millis() - g_last_reconnect_ms >= config::kWifiReconnectIntervalMs) {
      g_last_reconnect_ms = millis();
      if (wifiReconnect()) {
        g_wifi_down_since = 0;
        showRadarIfConnected();
      }
    }
  } else {
    g_wifi_down_since = 0;
    if (!g_radar_visible) {
      showRadarIfConnected();
    } else if (services::adsb::consumeUpdate()) {
      ui::radarDisplayRefreshAircraft();
    } else if (location_moved) {
      ui::radarDisplayDraw();
    }
  }

  delay(10);
}
