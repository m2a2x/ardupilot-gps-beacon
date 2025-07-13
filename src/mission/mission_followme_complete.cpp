#include "mission_followme_complete.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"
#include "conf.h"
#include "log_proxy.h"  // For logging

void FollowMeCompleteMission::start() {
    LogProxy::log("AutoMission: Starting auto mission");
    currentState = ARM_DRONE;
    stateStartTime = millis();
    lastPositionSend = 0;
    
    // Reset verification flags
    armCommandSent = false;
    modeCommandSent = false;
    takeoffCommandSent = false;
    armAckReceived = false;
    modeAckReceived = false;
    takeoffAckReceived = false;
    
    resetUpdateCount();
    LogProxy::log("Mission started: Will arm, takeoff to 10m, then follow");
}

void FollowMeCompleteMission::update() {
    switch (currentState) {
        case ARM_DRONE: {
            if (!armCommandSent) {
                LogProxy::log("Step 1: Sending arm command...");
                send_arm_command(true);
                armCommandSent = true;
                stateStartTime = millis();
            }
            // Wait for arm acknowledgment or timeout
            if (armAckReceived) {
                LogProxy::log("✓ Arm command acknowledged successfully");
                currentState = SET_GUIDED_MODE;
                stateStartTime = millis();
            } else if (millis() - stateStartTime >= 5000) { // 5 second timeout
                LogProxy::log("⚠ Arm command timeout, retrying...");
                armCommandSent = false; // Retry
            }
            break;
        }
        case SET_GUIDED_MODE: {
            if (!modeCommandSent) {
                LogProxy::log("Step 2: Setting GUIDED mode...");
                send_set_mode("GUIDED");
                modeCommandSent = true;
                stateStartTime = millis();
            }
            // Wait for mode acknowledgment or timeout
            if (modeAckReceived) {
                LogProxy::log("✓ GUIDED mode set successfully");
                currentState = TAKEOFF;
                stateStartTime = millis();
            } else if (millis() - stateStartTime >= 3000) { // 3 second timeout
                LogProxy::log("⚠ Mode command timeout, retrying...");
                modeCommandSent = false; // Retry
            }
            break;
        }
        case TAKEOFF: {
            if (!takeoffCommandSent) {
                LogProxy::log("Step 3: Sending takeoff command to 10m...");
                send_takeoff_command(TAKEOFF_ALTITUDE);
                takeoffCommandSent = true;
                stateStartTime = millis();
            }
            // Wait for takeoff acknowledgment or timeout
            if (takeoffAckReceived) {
                LogProxy::log("✓ Takeoff command acknowledged");
                currentState = WAIT_TAKEOFF_COMPLETE;
                stateStartTime = millis();
            } else if (millis() - stateStartTime >= 5000) { // 5 second timeout
                LogProxy::log("⚠ Takeoff command timeout, retrying...");
                takeoffCommandSent = false; // Retry
            }
            break;
        }
        case WAIT_TAKEOFF_COMPLETE: {
            float current_alt = getAltitude();
            LogProxy::log("Waiting for takeoff completion... Current altitude: " + String(current_alt, 1) + "m");
            if (current_alt >= TAKEOFF_ALTITUDE * 0.8) { // 80% of target altitude
                LogProxy::log("✓ Takeoff completed! Altitude: " + String(current_alt, 1) + "m");
                currentState = FOLLOW_MODE;
                stateStartTime = millis();
                LogProxy::log("Step 4: Starting follow-me mode...");
            } else if (millis() - stateStartTime >= 30000) { // 30 second timeout for takeoff
                LogProxy::log("⚠ Takeoff timeout - proceeding to follow mode anyway");
                currentState = FOLLOW_MODE;
                stateStartTime = millis();
            }
            break;
        }
        case FOLLOW_MODE: {
            if (millis() - lastPositionSend >= POSITION_SEND_INTERVAL && gpsHasFix()) {
                double target_lat = getLatitude();
                double target_lon = getLongitude();
                float target_alt = getAltitude();
                double offset_lat, offset_lon;
                calculate_offset_position(target_lat, target_lon, FOLLOW_OFFSET, offset_lat, offset_lon);
                send_position_target(offset_lat, offset_lon, target_alt);
                updateCount++;  // Increment counter for successful update
                lastPositionSend = millis();
                if (updateCount % 10 == 0) {
                    LogProxy::log("Following... Updates: " + String(updateCount) + ", Alt: " + String(target_alt, 1) + "m, Pos: " + String(offset_lat, 6) + "," + String(offset_lon, 6));
                }
            }
            break;
        }
        case COMPLETE: {
            // Mission is complete, nothing more to do
            break;
        }
    }
}

void FollowMeCompleteMission::stop() {
    LogProxy::log("AutoMission: Stopping mission");
    
    // Send disarm command for safety
    send_arm_command(false);
    
    currentState = COMPLETE;
    LogProxy::log("Mission stopped and drone disarmed");
}


void FollowMeCompleteMission::onCommandAck(uint16_t command, uint8_t result) {
    LogProxy::log("Command ACK received: command=" + String(command) + ", result=" + String(result));
    
    switch (command) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (result == MAV_RESULT_ACCEPTED) {
                armAckReceived = true;
                LogProxy::log("✓ Arm command accepted by drone");
            } else {
                LogProxy::log("⚠ Arm command failed with result: " + String(result));
            }
            break;
            
        case MAV_CMD_DO_SET_MODE:
            if (result == MAV_RESULT_ACCEPTED) {
                modeAckReceived = true;
                LogProxy::log("✓ Mode change accepted by drone");
            } else {
                LogProxy::log("⚠ Mode change failed with result: " + String(result));
            }
            break;
            
        case MAV_CMD_NAV_TAKEOFF:
            if (result == MAV_RESULT_ACCEPTED) {
                takeoffAckReceived = true;
                LogProxy::log("✓ Takeoff command accepted by drone");
            } else {
                LogProxy::log("⚠ Takeoff command failed with result: " + String(result));
            }
            break;
    }
}

const char* FollowMeCompleteMission::getCurrentStateName() const {
    switch (currentState) {
        case ARM_DRONE: return "ARM_DRONE";
        case WAIT_ARM_ACK: return "WAIT_ARM_ACK";
        case SET_GUIDED_MODE: return "SET_GUIDED_MODE";
        case WAIT_MODE_ACK: return "WAIT_MODE_ACK";
        case TAKEOFF: return "TAKEOFF";
        case WAIT_TAKEOFF_ACK: return "WAIT_TAKEOFF_ACK";
        case WAIT_TAKEOFF_COMPLETE: return "WAIT_TAKEOFF_COMPLETE";
        case FOLLOW_MODE: return "FOLLOW_MODE";
        case COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
} 