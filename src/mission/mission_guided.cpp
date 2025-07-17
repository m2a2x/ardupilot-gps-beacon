#include "mission_guided.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"
#include "utils.h"

void GuidedMission::start() {
    send_set_mode("GUIDED");
    resetUpdateCount();  // Reset counter when starting
}

void GuidedMission::update() {
    static unsigned long lastPositionSend = 0;
    const unsigned long POSITION_SEND_INTERVAL = 1000; // 1 second interval
    
    // Check if it's time to send position target and GPS has valid fix
    if (millis() - lastPositionSend >= POSITION_SEND_INTERVAL && gpsHasFix()) {
        if (executeFollowMeLogic(3.0, true, "Guided")) {
            updateCount++;  // Increment counter for successful update
            lastPositionSend = millis();
        }
    }
}

void GuidedMission::stop() {
    send_set_mode("GUIDED");
} 