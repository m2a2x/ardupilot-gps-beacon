#include "mission_goto.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"

void GoToMission::start() {
    send_set_mode("GUIDED");
    
    // Wait for GPS fix before setting target
    if (gpsHasFix()) {
        // Get current position
        target_lat = getLatitude();
        target_lon = getLongitude();
        float current_alt = getAltitude();
        
        // Calculate target position (10m north, 5m east, 5m higher)
        // const double NORTH_OFFSET = 0.000090;  // 10 meters north
        // const double EAST_OFFSET = 0.000045;   // 5 meters east
        
        // target_lat = current_lat + NORTH_OFFSET;  // Move north
        // target_lon = current_lon + EAST_OFFSET;   // Move east
        target_alt = current_alt + 2.0;  // 2 meters higher
        
        target_set = true;
        update();
        
        Serial.println("Lat=" + String(target_lat, 6) + " Lon=" + String(target_lon, 6) + " Alt=" + String(target_alt));
    } else {
        Serial.println("No GPS fix available, cannot set target");
        target_set = false;
    }
}

void GoToMission::update() {
    static unsigned long lastPositionSend = 0;
    const unsigned long POSITION_SEND_INTERVAL = 1000; // 1 second interval
    
    // Only send position target if we have a valid target and GPS fix
    if (target_set && gpsHasFix() && millis() - lastPositionSend >= POSITION_SEND_INTERVAL) {
        // Send the fixed target coordinates (set once in start())
        send_position_target(target_lat, target_lon, target_alt);
        lastPositionSend = millis();
    }
}

void GoToMission::stop() {
    target_set = false;
    send_set_mode("GUIDED");
} 