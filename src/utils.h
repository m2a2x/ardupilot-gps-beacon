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
 * Update current mission
 * This function should be called regularly in the main loop
 * to update the currently active mission
 */
void updateCurrentMission();

/**
 * Get the name of the currently active flight mode
 * @return String representing the active flight mode, or empty string if none
 */
String getActiveFlightMode();

/**
 * Get mission status display
 * @param lines Vector to store mission display lines
 */
void getMissionStatusDisplay(std::vector<String>& lines);

// External declarations
extern Mission* currentMission;

// Mission callback for command acknowledgments
void handleMissionCommandAck(uint16_t command, uint8_t result);

/**
 * Get the update count from the currently active mission
 * @return Update count as string, or "0" if no mission is active
 */
String getCurrentMissionUpdateCount();

/**
 * Execute follow-me logic: send position target with offset behind beacon
 * @param offset_meters Distance behind beacon in meters (positive = behind)
 * @param gps_valid Whether GPS has valid fix
 * @param log_prefix Optional prefix for logging messages
 * @return true if position was sent successfully, false otherwise
 */
bool executeFollowMeLogic(float offset_meters = 3.0, bool gps_valid = true, const String& log_prefix = ""); 