#include "mission_guided.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"

void GuidedMission::start() {
    send_set_mode("GUIDED");
    resetUpdateCount();  // Reset counter when starting
}

void GuidedMission::update() {
    static unsigned long lastPositionSend = 0;
    const unsigned long POSITION_SEND_INTERVAL = 1000; // 1 second interval
    
    // Check if it's time to send position target and GPS has valid fix
    if (millis() - lastPositionSend >= POSITION_SEND_INTERVAL && gpsHasFix()) {
        double target_lat = getLatitude();
        double target_lon = getLongitude();
        float target_alt = getAltitude();
        
        // Simple offset: 3 meters behind = subtract from latitude (move south)
        // 3 meters ≈ 0.000027 degrees (3/111000)
        const double OFFSET_DEGREES = 0.000027;
        double offset_lat = target_lat - OFFSET_DEGREES;  // Move south (behind)
        double offset_lon = target_lon;  // Same longitude
        
        // Send position target with offset coordinates (3 meters behind)
        send_position_target(offset_lat, offset_lon, target_alt);
        
        updateCount++;  // Increment counter for successful update
        lastPositionSend = millis();
    }
}

void GuidedMission::stop() {
    send_set_mode("GUIDED");
} 