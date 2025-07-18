#include "mission_followme.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"
#include "conf.h"
#include "log_proxy.h"

void FollowMeMission::start() {
    // Set mode to GUIDED first
    send_set_mode("GUIDED");
    
    // Wait 5 seconds
    delay(5000);
    
    // Then set to FOLLOW mode
    send_set_mode("FOLLOW");
    
    resetUpdateCount();  // Reset counter when starting
}

void FollowMeMission::update() {
    if (isRunning && gpsHasFix()) {
        send_set_mode("FOLLOW");
        double lat = getLatitude();
        double lon = getLongitude();
        float alt = getAltitude() + FOLLOW_ALT;
        sendFollowTargetLatLon(millis(), lat, lon, alt);
        
        updateCount++;  // Increment counter for successful update
    }
}

void FollowMeMission::stop() {
    // Set drone to LOITER mode for safe hovering
    send_set_mode_command("LOITER");
    LogProxy::log("Setting drone to LOITER mode for safe hovering");
}

void FollowMeMission::onStart() {
    // Mission-specific start logic
    LogProxy::log("FollowMeMission: onStart - preparing follow me mode");
}

void FollowMeMission::onStop() {
    // Mission-specific stop logic
    LogProxy::log("FollowMeMission: onStop - follow me mode stopped");
}

void FollowMeMission::onRTL() {
    // Mission-specific RTL logic
    // Set drone to RTL mode for return to launch
    send_set_mode_command("RTL");
    LogProxy::log("FollowMeMission: Setting drone to RTL mode for return to launch");
} 