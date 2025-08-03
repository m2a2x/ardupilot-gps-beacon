#include "mission_guided.h"
#include "mavlink_cmds.h"
#include "gps.h"
#include "utils.h"
#include "log_proxy.h"
#include "gps_utils.h"  // For GPS calculation functions

void GuidedMission::start() {
    // This is now handled by BaseMission::handleMenuAction
    // The actual start logic is in onStart()
}

void GuidedMission::update() {
    if (isRunning) {
        static unsigned long lastPositionSend = 0;
        const unsigned long POSITION_SEND_INTERVAL = 1000; // 1 second interval
        
        // Check if it's time to send position target and GPS has valid fix
        if (millis() - lastPositionSend >= POSITION_SEND_INTERVAL && gpsHasFix()) {
            if (executeFollowMeLogic(3.0, 0.0, true, "Guided")) {
                updateCount++;  // Increment counter for successful update
                lastPositionSend = millis();
            }
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
bool GuidedMission::executeFollowMeLogic(float offset_meters, float altitude_offset_meters, bool gps_valid, const String& log_prefix) {
    if (!gps_valid) {
        return false;
    }
    
    double target_lat = getLatitude();
    double target_lon = getLongitude();
    float target_alt = 5.0;
    
    // Calculate offset position using GPS utility function
    // Note: For stationary targets, "behind" is ambiguous. Using North (0 radians) as default.
    // For moving targets, this should be updated to use the target's heading/bearing.
    double offset_lat, offset_lon;
    calculateOffsetPosition(target_lat, target_lon, offset_meters, 0.0, offset_lat, offset_lon);
    
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

void GuidedMission::stop() {
    // This is now handled by BaseMission::handleMenuAction
    // The actual stop logic is in onStop()
}

void GuidedMission::onStart() {
    // Mission-specific start logic
    send_set_mode_command("GUIDED");
    resetUpdateCount();  // Reset counter when starting
    LogProxy::log("Starting Guided Mode...");
}

void GuidedMission::onStop() {
    // Mission-specific stop logic
    // Set drone to LOITER mode for safe hovering
    send_set_mode_command("LOITER");
    LogProxy::log("Setting drone to LOITER mode for safe hovering");
}

void GuidedMission::onRTL() {
    // Mission-specific RTL logic
    // Set drone to RTL mode for return to launch
    send_set_mode_command("RTL");
} 