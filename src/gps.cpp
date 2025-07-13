#include "gps.h"
#include "conf.h"  // For GPS_RX_PIN and GPS_TX_PIN
#include "log_proxy.h"  // For logging

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
unsigned long lastGPSUpdate = 0;
static const double MIN_ALTITUDE = 3.0;  // Minimum altitude in meters

bool setupGPS() {
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  return gpsSerial.available();
}

bool updateGPS() {
  bool newData = false;
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      newData = true;
      lastGPSUpdate = millis();
    }
  }
  return newData;
}

bool gpsHasFix() {
  return gps.location.isValid() && (millis() - lastGPSUpdate < GPS_TIMEOUT_MS);
}

double getLatitude() {
  return gps.location.isValid() ? gps.location.lat() : 0.0;
}

double getLongitude() {
  return gps.location.isValid() ? gps.location.lng() : 0.0;
}

double getAltitude() {
  if (!gps.altitude.isValid()) {
    return MIN_ALTITUDE;
  }
  
  double alt = gps.altitude.meters();
  if (alt < MIN_ALTITUDE) {
    return MIN_ALTITUDE;
  }
  
  return alt;
}

int getSatelliteCount() {
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}

bool isGPSStale() {
  return (millis() - lastGPSUpdate > GPS_TIMEOUT_MS);
} 