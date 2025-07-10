#pragma once
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "conf.h"  // For GPS_RX_PIN and GPS_TX_PIN

// GPS timeout configuration
const unsigned long GPS_TIMEOUT_MS = 5000;  // 5 seconds timeout

// External declarations
extern unsigned long lastGPSUpdate;  // Last GPS update timestamp

/**
 * Initialize GPS module
 * @return true if GPS serial port was opened successfully
 */
bool setupGPS();

/**
 * Update GPS data from serial port
 * @return true if new data was received
 */
bool updateGPS();

/**
 * Check if GPS has a valid fix
 * @return true if GPS has valid location data
 */
bool gpsHasFix();

/**
 * Get current latitude
 * @return latitude in degrees, or 0.0 if invalid
 */
double getLatitude();

/**
 * Get current longitude
 * @return longitude in degrees, or 0.0 if invalid
 */
double getLongitude();

/**
 * Get current altitude
 * @return altitude in meters, or 0.0 if invalid
 */
double getAltitude();

/**
 * Get number of satellites in view
 * @return number of satellites, or 0 if invalid
 */
int getSatelliteCount();

/**
 * Check if GPS data is stale
 * @return true if no GPS data received within timeout period
 */
bool isGPSStale();
