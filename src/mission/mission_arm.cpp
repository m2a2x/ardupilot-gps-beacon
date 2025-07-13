#include "mission_arm.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "log_proxy.h"  // For logging

void ArmMission::start() {
    LogProxy::log("ArmMission: start");
    currentState = ARM;
    stateStartTime = millis();
    send_set_mode("GUIDED");
    LogProxy::log("Starting " + String(shouldArm ? "arm" : "disarm") + " mission");
}

void ArmMission::update() {
    switch (currentState) {
        case ARM:
            // Send arm/disarm command
            send_arm_command(shouldArm, true);
            currentState = WAIT_AFTER_ARM;
            stateStartTime = millis();
            LogProxy::log(String(shouldArm ? "Arm" : "Disarm") + " command sent, waiting 3 seconds");
            break;
            
        case WAIT_AFTER_ARM:
            if (millis() - stateStartTime >= 3000) { // Wait 3 seconds
                currentState = COMPLETE;
                stateStartTime = millis();
                LogProxy::log(String(shouldArm ? "Arm" : "Disarm") + " mission completed");
            }
            break;
            
        case COMPLETE:
            // Mission is complete, nothing more to do
            break;
    }
}

void ArmMission::stop() {
    LogProxy::log("ArmMission: stop");
    // Mission completed, no cleanup needed
} 