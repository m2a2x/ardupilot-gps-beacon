#include "utils.h"
#include "conf.h"
#include "gps.h"
#include "gps_utils.h"  // For GPS calculation functions
#include "mission/mission_followme_complete.h"
#include "mavlink_cmds.h"
#include "log_proxy.h"  // For logging
#include <mavlink/v2.0/common/mavlink.h>
#include <algorithm>  // For std::remove_if
#include <set>        // For std::set
#include <cstring>    // For strcmp
#include <math.h>     // For cos, sin functions

#include "menu/flight_modes.h"  // For flight mode functions

// Mission pointer
Mission* currentMission = nullptr;

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
                      float dt, float &vx, float &vy, float &vz) {
  // Calculate horizontal distance using GPS utility function
  float distance = calculateGPSDistance(lat1, lon1, lat2, lon2);
  
  // Calculate bearing to get direction
  float bearing = calculateGPSBearing(lat1, lon1, lat2, lon2);
  
  // Calculate velocity components
  vx = distance * cos(bearing) / dt;
  vy = distance * sin(bearing) / dt;
  vz = (alt2 - alt1) / dt;
}

/**
 * Update current mission
 * This function should be called regularly in the main loop
 * to update the currently active mission
 */
void updateCurrentMission() {
  if (currentMission != nullptr) {
    currentMission->update();
  }
}

/**
 * Get mission status display
 * @param lines Vector to store mission display lines
 */
void getMissionStatusDisplay(std::vector<String>& lines) {
  lines.push_back("== MISSION STATUS ==");
  if (currentMission) {
    lines.push_back(String("Active Mission: ") + currentMission->getName());
    lines.push_back(String("Updates: ") + getCurrentMissionUpdateCount());
  } else {
    lines.push_back("No active mission");
  }
  lines.push_back("");
}

/**
 * Get the update count from the currently active mission
 * @return Update count as string, or "0" if no mission is active
 */
String getCurrentMissionUpdateCount() {
    if (currentMission != nullptr) {
        return String(currentMission->getUpdateCount());
    }
    return "0";
}

/**
 * Handle command acknowledgment for the current mission
 * @param command The command that was acknowledged
 * @param result The result of the command
 */
void handleMissionCommandAck(uint16_t command, uint8_t result) {
    if (currentMission != nullptr) {
        // Check if the current mission is a FollowMeCompleteMission using string comparison
        if (strcmp(currentMission->getType(), "FollowMeComplete") == 0) {
            // Cast to FollowMeCompleteMission and call the callback
            FollowMeCompleteMission* autoMission = static_cast<FollowMeCompleteMission*>(currentMission);
            autoMission->onCommandAck(command, result);
        }
    }
}

/**
 * Execute follow-me logic: send position target with offset behind beacon
 * @param offset_meters Distance behind beacon in meters (positive = behind)
 * @param gps_valid Whether GPS has valid fix
 * @param log_prefix Optional prefix for logging messages
 * @return true if position was sent successfully, false otherwise
 */
bool executeFollowMeLogic(float offset_meters, float altitude_offset_meters, bool gps_valid, const String& log_prefix) {
    if (!gps_valid) {
        return false;
    }
    
    double target_lat = getLatitude();
    double target_lon = getLongitude();
    float target_alt = getAltitude();
    
    // Calculate offset position using GPS utility function
    double offset_lat, offset_lon;
    calculateOffsetPosition(target_lat, target_lon, offset_meters, offset_lat, offset_lon);
    
    // Apply altitude offset
    float final_alt = target_alt + altitude_offset_meters;
    
    // Send position target with offset coordinates and altitude
    send_position_target(offset_lat, offset_lon, final_alt);
    
    // Log the action if prefix is provided
    if (log_prefix.length() > 0) {
        String logMsg = log_prefix + " Following... Target: " + String(offset_lat, 6) + "," + String(offset_lon, 6) + 
                       " (" + String(offset_meters, 1) + "m behind, " + String(altitude_offset_meters, 1) + "m alt offset)";
        LogProxy::log(logMsg);
    }
    
    return true;
} 