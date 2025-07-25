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


