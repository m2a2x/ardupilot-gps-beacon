#include "mission_follow.h"
#include "mavlink_cmds.h"
#include "gps.h"
#include "log_proxy.h"
#include "drone_status.h"

// External declarations
extern DroneStatus droneStatus;

void FollowMission::start() {
    // Call parent's start method
    FollowMeCompleteMission::start();
    
    targetAlt = targetAlt - 3;
}

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
        LogProxy::log("No GPS fix");
        return;
    }
    
    // Get current beacon position (this vehicle's position)
    double beaconLat = getFilteredLatitude();
    double beaconLon = getFilteredLongitude();
    
    // Convert relative altitude to MSL altitude
    // targetAlt is relative to home, but FOLLOW_TARGET needs MSL altitude
    float followAltitude = targetAlt + 1;
    if (droneStatus.isHomePositionValid()) {
        // Use home altitude directly if available
        followAltitude = droneStatus.home_altitude_amsl + followAltitude;
    } else if (droneStatus.isAltitudeValid()) {
        // Fallback: calculate home altitude from drone position
        float homeAmsl = droneStatus.altitude_amsl - droneStatus.altitude_relative;
        followAltitude = homeAmsl + followAltitude;
    }
    // Send FOLLOW_TARGET message using the proper MAVLink function with capabilities
    // This allows other vehicles to follow this one
    sendFollowTargetLatLonWithCapabilities(millis(), beaconLat, beaconLon, ceil(followAltitude), CAPABILITIES_POS);
}