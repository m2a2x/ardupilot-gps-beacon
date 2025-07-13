#pragma once
#include <Arduino.h>  // For millis(), PI, cos(), sin(), random()

/**
 * Initialize GPS simulation
 * @return true if simulation was initialized successfully
 */
bool setupGPSSimulation();

/**
 * Update GPS simulation data
 * @return true if new simulation data was generated
 */
bool updateGPSSimulation();

/**
 * Get simulated latitude
 * @return simulated latitude in degrees
 */
double getSimulatedLatitude();

/**
 * Get simulated longitude
 * @return simulated longitude in degrees
 */
double getSimulatedLongitude();

/**
 * Get simulated altitude
 * @return simulated altitude in meters
 */
double getSimulatedAltitude();

/**
 * Get simulated satellite count
 * @return simulated number of satellites
 */
int getSimulatedSatelliteCount();

/**
 * Check if simulation has a valid fix
 * @return true (simulation always has valid data)
 */
bool simulationHasFix();

/**
 * Reset simulation to initial position
 */
void resetSimulation(); 