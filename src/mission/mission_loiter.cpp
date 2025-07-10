#include "mission_loiter.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"

void LoiterMission::start() {
    Serial.println("LoiterMission: start");
    currentState = SET_MODE;
    stateStartTime = millis();
    Serial.println("Starting loiter mission sequence");
}

void LoiterMission::update() {
    switch (currentState) {
        case SET_MODE:
            // Set mode to GUIDED first
            send_set_mode("GUIDED");
            currentState = WAIT_AFTER_MODE;
            stateStartTime = millis();
            Serial.println("Set mode to GUIDED, waiting 3 seconds");
            break;
            
        case WAIT_AFTER_MODE:
            if (millis() - stateStartTime >= 3000) { // Wait 3 seconds
                currentState = ARM;
                stateStartTime = millis();
                Serial.println("Mode set, proceeding to arm");
            }
            break;
            
        case ARM:
            // Send arm command
            send_arm_command(true);
            currentState = WAIT_AFTER_ARM;
            stateStartTime = millis();
            Serial.println("Arm command sent, waiting 3 seconds");
            break;
            
        case WAIT_AFTER_ARM:
            if (millis() - stateStartTime >= 3000) { // Wait 3 seconds
                currentState = TAKEOFF;
                stateStartTime = millis();
                Serial.println("Armed, proceeding to takeoff");
            }
            break;
            
        case TAKEOFF:
            executeTakeoff();
            break;
            
        case LOITER:
            send_set_mode("LOITER");
            break;
    }
}

void LoiterMission::stop() {
    Serial.println("LoiterMission: stop");
    // Optionally send a mode change or cleanup
}

void LoiterMission::executeTakeoff() {
    unsigned long currentMillis = millis();
    
    // Send takeoff command
    send_takeoff_command(takeoff_alt);
    
    if (currentMillis - stateStartTime > 10000) { // Wait 10 seconds for takeoff
        currentState = LOITER;
        stateStartTime = currentMillis;
        Serial.println("Moving to LOITER state");
        // Loiter mode already set at start, no need to set it again
    }
} 