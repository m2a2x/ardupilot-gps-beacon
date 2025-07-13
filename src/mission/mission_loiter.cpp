#include "mission_loiter.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "log_proxy.h"  // For logging
#include "gps.h"

void LoiterMission::start() {
    LogProxy::log("LoiterMission: start");
    currentState = SET_MODE;
    stateStartTime = millis();
    LogProxy::log("Starting loiter mission sequence");
}

void LoiterMission::update() {
    switch (currentState) {
        case SET_MODE:
            // Set mode to GUIDED first
            send_set_mode("GUIDED");
            currentState = WAIT_AFTER_MODE;
            stateStartTime = millis();
            LogProxy::log("Set mode to GUIDED, waiting 3 seconds");
            break;
            
        case WAIT_AFTER_MODE:
            if (millis() - stateStartTime >= 3000) { // Wait 3 seconds
                currentState = ARM;
                stateStartTime = millis();
                LogProxy::log("Mode set, proceeding to arm");
            }
            break;
            
        case ARM:
            // Send arm command
            send_arm_command(true);
            currentState = WAIT_AFTER_ARM;
            stateStartTime = millis();
            LogProxy::log("Arm command sent, waiting 3 seconds");
            break;
            
        case WAIT_AFTER_ARM:
            if (millis() - stateStartTime >= 3000) { // Wait 3 seconds
                currentState = TAKEOFF;
                stateStartTime = millis();
                LogProxy::log("Armed, proceeding to takeoff");
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
    LogProxy::log("LoiterMission: stop");
    // Optionally send a mode change or cleanup
}

void LoiterMission::executeTakeoff() {
    unsigned long currentMillis = millis();
    
    // Send takeoff command
    send_takeoff_command(takeoff_alt);
    
    if (currentMillis - stateStartTime > 10000) { // Wait 10 seconds for takeoff
        currentState = LOITER;
        stateStartTime = currentMillis;
        LogProxy::log("Moving to LOITER state");
        // Loiter mode already set at start, no need to set it again
    }
} 