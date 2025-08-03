#pragma once
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "conf.h"  // For GPS_RX_PIN and GPS_TX_PIN
#include "gps_simulate.h"  // For GPS simulation functionality

// GPS timeout configuration
const unsigned long GPS_TIMEOUT_MS = 5000;  // 5 seconds timeout

// Moving average filter configuration
const int GPS_AVERAGE_WINDOW_SIZE = 5;  // Number of samples to keep for averaging
const float GPS_MAX_JUMP_METERS = 5.0;  // Maximum allowed jump in meters (outlier rejection)

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
 * Apply moving average filtering to current GPS data (called internally during updateGPS)
 */
void applyMovingAverageFiltering();

/**
 * Check if GPS has a valid fix
 * @return true if GPS has valid location data
 */
bool gpsHasFix();

/**
 * Get current latitude (raw, unfiltered)
 * @return latitude in degrees, or 0.0 if invalid
 */
double getLatitude();

/**
 * Get current longitude (raw, unfiltered)
 * @return longitude in degrees, or 0.0 if invalid
 */
double getLongitude();

/**
 * Get filtered latitude using moving average
 * @return filtered latitude in degrees, or 0.0 if invalid
 */
double getFilteredLatitude();

/**
 * Get filtered longitude using moving average
 * @return filtered longitude in degrees, or 0.0 if invalid
 */
double getFilteredLongitude();

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

/**
 * Enable or disable GPS simulation mode
 * @param enable true to enable simulation, false to use real GPS
 */
void setGPSSimulation(bool enable);

/**
 * Check if GPS simulation mode is enabled
 * @return true if simulation mode is active
 */
bool isGPSSimulationEnabled();

/**
 * Reset moving average filters (useful when GPS fix is lost and regained)
 */
void resetGPSFilters();

/**
 * Get filter status information
 * @return true if filters are initialized and working properly
 */
bool getFilterStatus();

/**
 * Check if GPS is valid
 * @return true if GPS is valid
 */
bool isGPSValid();