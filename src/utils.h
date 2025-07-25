#pragma once
#include <Arduino.h>
#include <vector>
#include <mavlink/v2.0/common/mavlink.h>
#include "mission/mission.h"

/**
 * Calculate velocity between two GPS points
 * @param lat1 First latitude
 * @param lon1 First longitude
 * @param alt1 First altitude
 * @param lat2 Second latitude
 * @param lon2 Second longitude
 * @param alt2 Second altitude
 * @param dt Time difference in seconds
 * @param vx Output X velocity (m/s)
 * @param vy Output Y velocity (m/s)
 * @param vz Output Z velocity (m/s)
 */
void calculateVelocity(double lat1, double lon1, float alt1,
                      double lat2, double lon2, float alt2,
                      float dt, float &vx, float &vy, float &vz);
/**
 * Get the name of the currently active flight mode
 * @return String representing the active flight mode, or empty string if none
 */
String getActiveFlightMode();



// External declarations
extern Mission* currentMission;

