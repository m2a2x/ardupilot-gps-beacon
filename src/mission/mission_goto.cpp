#include "mission_goto.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"
#include "log_proxy.h"  // For logging

void GoToMission::onStart() {
    static unsigned long lastPositionSend = 0;
    const unsigned long POSITION_SEND_INTERVAL = 1000; // 1 second interval
    send_set_mode_command("GUIDED");
    
    // Wait for GPS fix before setting target
    if (gpsHasFix()) {
        // Get current position
        double target_lat = getFilteredLatitude();
        double target_lon = getFilteredLongitude();
        float target_alt = 5.0;
        
        // Send the fixed target coordinates (set once in start())
        send_position_target(target_lat, target_lon, target_alt);
        lastPositionSend = millis();
    }
}

void GoToMission::onStop() {
    send_set_mode_command("LOITER");
}

void GoToMission::onRTL() {
    send_set_mode_command("RTL");
}
