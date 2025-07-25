#include "gps.h"
#include "conf.h"
#include "log_proxy.h"
#include "gps_simulate.h"

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
unsigned long lastGPSUpdate = 0;
static const double MIN_ALTITUDE = 2.0;  // Minimum altitude in meters

// Simulation control
static bool isSimulate = true;

bool setupGPS() {
  if (isSimulate) {
    lastGPSUpdate = millis();
    return setupGPSSimulation();
  }
  
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  return gpsSerial.available();
}

bool updateGPS() {
  if (isSimulate) {
    bool newData = updateGPSSimulation();
    if (newData) {
      lastGPSUpdate = millis();
    }
    return newData;
  }
  
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
  if (isSimulate) {
    return simulationHasFix();
  }
  return gps.location.isValid() && (millis() - lastGPSUpdate < GPS_TIMEOUT_MS);
}

double getLatitude() {
  if (isSimulate) {
    return getSimulatedLatitude();
  }
  return gps.location.isValid() ? gps.location.lat() : 0.0;
}

double getLongitude() {
  if (isSimulate) {
    return getSimulatedLongitude();
  }
  return gps.location.isValid() ? gps.location.lng() : 0.0;
}

double getAltitude() {
  if (isSimulate) {
    return getSimulatedAltitude();
  }
  
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
  if (isSimulate) {
    return getSimulatedSatelliteCount();
  }
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}

bool isGPSStale() {
  return (millis() - lastGPSUpdate > GPS_TIMEOUT_MS);
}

void setGPSSimulation(bool enable) {
  isSimulate = enable;
  if (enable) {
    resetSimulation();
    lastGPSUpdate = millis();
  }
}

bool isGPSSimulationEnabled() {
  return isSimulate;
} 