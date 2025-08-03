#include "gps.h"
#include "conf.h"
#include "log_proxy.h"
#include "gps_simulate.h"
#include <math.h>

// Simulation control
#if DEBUG
static bool isSimulate = true;
#else
static bool isSimulate = false;
#endif

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
unsigned long lastGPSUpdate = 0;



// Moving average filter buffers for latitude and longitude
static double latBuffer[GPS_AVERAGE_WINDOW_SIZE];
static double lonBuffer[GPS_AVERAGE_WINDOW_SIZE];
static int bufferIndex = 0;
static int bufferCount = 0;

// Filter state tracking
static bool filtersInitialized = false;
static double lastFilteredLat = 0.0;
static double lastFilteredLon = 0.0;

// Cached filtered values (updated during GPS reading)
static double cachedFilteredLat = 0.0;
static double cachedFilteredLon = 0.0;
static bool hasValidFilteredData = false;

// Helper function to calculate distance between two GPS coordinates in meters
double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  // Validate input coordinates
  if (lat1 < -90.0 || lat1 > 90.0 || lon1 < -180.0 || lon1 > 180.0 ||
      lat2 < -90.0 || lat2 > 90.0 || lon2 < -180.0 || lon2 > 180.0) {
    return -1.0; // Invalid coordinates
  }
  
  // Check for identical coordinates to avoid division by zero
  if (lat1 == lat2 && lon1 == lon2) {
    return 0.0;
  }
  
  const double R = 6371000; // Earth's radius in meters
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  double a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// Helper function to calculate moving average
double calculateMovingAverage(double arr[], int size) {
  if (size == 0) return 0.0;
  
  double sum = 0.0;
  int validCount = 0;
  
  for (int i = 0; i < size; i++) {
    // Only include valid coordinates in the average
    if (arr[i] >= -180.0 && arr[i] <= 180.0) {
      sum += arr[i];
      validCount++;
    }
  }
  
  return validCount > 0 ? sum / validCount : 0.0;
}

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
      // Apply filtering immediately when new GPS data is available
      applyMovingAverageFiltering();
    }
  }
  return newData;
}

// Apply moving average filtering to current GPS data
void applyMovingAverageFiltering() {
  if (!isGPSValid()) {
    hasValidFilteredData = false;
    return;
  }
  
  double rawLat = gps.location.lat();
  double rawLon = gps.location.lng();
  
  // Validate input coordinates
  if (rawLat < -90.0 || rawLat > 90.0 || rawLon < -180.0 || rawLon > 180.0) {
    hasValidFilteredData = false;
    return;
  }
  
  // Outlier rejection: check if the new reading is too far from the last filtered value
  if (filtersInitialized && hasValidFilteredData) {
    double distance = calculateDistanceMeters(lastFilteredLat, lastFilteredLon, rawLat, rawLon);
    if (distance < 0.0) {
      // Invalid coordinates in distance calculation
      return;
    }
    if (distance > GPS_MAX_JUMP_METERS) {
      return; // Skip this reading
    }
  }
  
  // Add new reading to buffer
  latBuffer[bufferIndex] = rawLat;
  lonBuffer[bufferIndex] = rawLon;
  bufferIndex = (bufferIndex + 1) % GPS_AVERAGE_WINDOW_SIZE;
  if (bufferCount < GPS_AVERAGE_WINDOW_SIZE) {
    bufferCount++;
  }
  
  // Calculate moving average if we have enough samples
  if (bufferCount >= 2) { // Need at least 2 samples for meaningful average
    double filteredLat = calculateMovingAverage(latBuffer, bufferCount);
    double filteredLon = calculateMovingAverage(lonBuffer, bufferCount);
    
    // Validate filtered results
    if (filteredLat < -90.0 || filteredLat > 90.0 || filteredLon < -180.0 || filteredLon > 180.0) {
      LogProxy::log("GPS: Filtered coordinates out of range, using raw values");
      filteredLat = rawLat;
      filteredLon = rawLon;
    }
    
    // Update cached and last values
    cachedFilteredLat = filteredLat;
    cachedFilteredLon = filteredLon;
    lastFilteredLat = filteredLat;
    lastFilteredLon = filteredLon;
    hasValidFilteredData = true;
    filtersInitialized = true;
  } else {
    // Not enough samples yet, use raw values
    cachedFilteredLat = rawLat;
    cachedFilteredLon = rawLon;
    lastFilteredLat = rawLat;
    lastFilteredLon = rawLon;
    hasValidFilteredData = true;
    filtersInitialized = true;
  }
}

bool gpsHasFix() {
  if (isSimulate) {
    return simulationHasFix();
  }
  return isGPSValid();
}

bool isGPSValid() {
  return gps.location.isValid() && gps.hdop.hdop() < 2.0 && gps.satellites.value() >= 5;
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

double getFilteredLatitude() {
  if (isSimulate) {
    return getSimulatedLatitude();
  }
  
  if (!hasValidFilteredData) {
    return lastFilteredLat;  // Return last known good filtered value
  }
  
  return cachedFilteredLat;
}

double getFilteredLongitude() {
  if (isSimulate) {
    return getSimulatedLongitude();
  }
  
  if (!hasValidFilteredData) {
    return lastFilteredLon;  // Return last known good filtered value
  }
  
  return cachedFilteredLon;
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

void resetGPSFilters() {
  // Reset filter state
  filtersInitialized = false;
  lastFilteredLat = 0.0;
  lastFilteredLon = 0.0;
  cachedFilteredLat = 0.0;
  cachedFilteredLon = 0.0;
  hasValidFilteredData = false;
  
  // Reset buffer
  bufferIndex = 0;
  bufferCount = 0;
  
  // Clear buffer arrays with invalid values to indicate uninitialized state
  for (int i = 0; i < GPS_AVERAGE_WINDOW_SIZE; i++) {
    latBuffer[i] = 999.0;  // Invalid latitude value
    lonBuffer[i] = 999.0;  // Invalid longitude value
  }
}

/**
 * Get filter status information
 * @return true if filters are initialized and working properly
 */
bool getFilterStatus() {
  return filtersInitialized && hasValidFilteredData && bufferCount >= 2;
} 