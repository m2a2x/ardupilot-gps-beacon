#include "mission_arm.h"
#include <Arduino.h>
#include "mavlink_cmds.h"

void ArmMission::start() {
    Serial.println("ArmMission: start");
    currentState = ARM;
    stateStartTime = millis();
    Serial.printf("Starting %s mission\n", shouldArm ? "arm" : "disarm");
}

void ArmMission::update() {
    switch (currentState) {
        case ARM:
            // Send arm/disarm command
            send_arm_command(shouldArm);
            currentState = WAIT_AFTER_ARM;
            stateStartTime = millis();
            Serial.printf("%s command sent, waiting 3 seconds\n", shouldArm ? "Arm" : "Disarm");
            break;
            
        case WAIT_AFTER_ARM:
            if (millis() - stateStartTime >= 3000) { // Wait 3 seconds
                currentState = COMPLETE;
                stateStartTime = millis();
                Serial.printf("%s mission completed\n", shouldArm ? "Arm" : "Disarm");
            }
            break;
            
        case COMPLETE:
            // Mission is complete, nothing more to do
            break;
    }
}

void ArmMission::stop() {
    Serial.println("ArmMission: stop");
    // Mission completed, no cleanup needed
} 