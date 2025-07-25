#include "mission_follow.h"
#include "mavlink_cmds.h"
#include "gps.h"
#include "log_proxy.h"
#include "drone_status.h"

// External declarations
extern DroneStatus droneStatus;

void FollowMission::handleInFollowMode() {
    // Send FOLLOW_TARGET messages at 10Hz (every 100ms) like the Lua script
    if (millis() - lastFollowTargetSend >= FOLLOW_UPDATE_INTERVAL) {
        send_set_mode_command("FOLLOW");
        sendFollowTargetMessage();
        lastFollowTargetSend = millis();
        updateCount++; // Increment counter for successful updates
    }
}

void FollowMission::sendFollowTargetMessage() {
    // Check if we have GPS fix
    if (!gpsHasFix()) {
        LogProxy::log("⚠ Follow Mission: No GPS fix, cannot send follow target");
        return;
    }
    
    // Get current beacon position (this vehicle's position)
    double beaconLat = getLatitude();
    double beaconLon = getLongitude();
    float beaconAlt = getAltitude();
    
    // Send FOLLOW_TARGET message using the proper MAVLink function with capabilities
    // This allows other vehicles to follow this one
    uint64_t timestamp = millis();
    sendFollowTargetLatLonWithCapabilities(timestamp, beaconLat, beaconLon, beaconAlt, FOLLOW_TARGET_CAPABILITIES_POS);
    
    // Log follow target broadcast (less frequently to avoid spam)
    static unsigned long lastLog = 0;
    if (millis() - lastLog >= 5000) { // Log every 5 seconds
        lastLog = millis();
    }
}