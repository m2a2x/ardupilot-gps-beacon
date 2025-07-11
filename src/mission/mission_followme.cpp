#include "mission_followme.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"
#include "conf.h"

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
    if (gpsHasFix()) {
        send_set_mode("FOLLOW");
        double lat = getLatitude();
        double lon = getLongitude();
        float alt = getAltitude() + FOLLOW_ALT;
        sendFollowTargetLatLon(millis(), lat, lon, alt);
        
        updateCount++;  // Increment counter for successful update
    }
}

void FollowMeMission::stop() {
    send_set_mode("GUIDED");
} 