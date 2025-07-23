#include "gps_simulate.h"
#include <Arduino.h>

// Simulation variables
static double simLatitude = 41.697391;   // Starting latitude
static double simLongitude = -8.808960;  // Starting longitude
static double simAltitude = 0.0;         // Starting altitude
static double simHeading = 0.0;          // Current heading in degrees
static unsigned long lastSimUpdate = 0;  // Last simulation update time
static const unsigned long SIM_UPDATE_INTERVAL = 1000; // Update every 1 second
static const double WALKING_SPEED = 0.00005; // Small movement per update (roughly 1-2 meters)
static const double MIN_ALTITUDE = 5.0;  // Minimum altitude in meters

bool setupGPSSimulation() {
  resetSimulation();
  return true;
}

bool updateGPSSimulation() {
  unsigned long currentTime = millis();
  if (currentTime - lastSimUpdate >= SIM_UPDATE_INTERVAL) {
    // Simulate walking movement
    simHeading += 5.0; // Change direction slightly
    if (simHeading >= 360.0) simHeading -= 360.0;
    
    // Convert heading to radians
    double headingRad = simHeading * PI / 180.0;
    
    // Calculate new position (simplified walking simulation)
    simLatitude += WALKING_SPEED * cos(headingRad);
    simLongitude += WALKING_SPEED * sin(headingRad);
    
    // Small altitude variation
    simAltitude += (random(-10, 10) / 100.0); // ±0.1m variation
    if (simAltitude < 0) simAltitude = 0;
    
    lastSimUpdate = currentTime;
    return true;
  }
  return false;
}

double getSimulatedLatitude() {
  return simLatitude;
}

double getSimulatedLongitude() {
  return simLongitude;
}

double getSimulatedAltitude() {
  return simAltitude < MIN_ALTITUDE ? MIN_ALTITUDE : simAltitude;
}

int getSimulatedSatelliteCount() {
  return 8; // Simulate 8 satellites
}

bool simulationHasFix() {
  return true; // Simulation always has valid data
}

void resetSimulation() {
  simLatitude = 41.697391;
  simLongitude = -8.808960;
  simAltitude = 0.0;
  simHeading = 0.0;
  lastSimUpdate = millis();
} 