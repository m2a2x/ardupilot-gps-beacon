#include "mission_loiter.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"
#include "conf.h"
#include "log_proxy.h"  // For logging

void LoiterMission::start() {
    LogProxy::log("LoiterMission: Starting loiter mission");
    currentState = ARM_DRONE;
    stateStartTime = millis();
    
    // Reset verification flags
    armCommandSent = false;
    modeCommandSent = false;
    takeoffCommandSent = false;
    armAckReceived = false;
    modeAckReceived = false;
    takeoffAckReceived = false;
    
    LogProxy::log("Mission started: Will arm, takeoff to 3m, then loiter");
}

void LoiterMission::update() {
    switch (currentState) {
        case ARM_DRONE: {
            if (!armCommandSent) {
                LogProxy::log("Step 1: Sending arm command...");
                send_arm_command(true, true);
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
                send_set_mode_command("GUIDED");
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
                // Check if we have GPS fix before sending takeoff command
                if (gpsHasFix()) {
                    double lat = getLatitude();
                    double lon = getLongitude();
                    LogProxy::log("Step 3: Sending takeoff command to " + String(TAKEOFF_ALTITUDE, 1) + "m...");
                    send_takeoff_command(TAKEOFF_ALTITUDE);
                    takeoffCommandSent = true;
                    stateStartTime = millis();
                } else {
                    static unsigned long lastGpsLog = 0;
                    if (millis() - lastGpsLog >= 2000) { // Log every 2 seconds to avoid spam
                        LogProxy::log("⚠ Waiting for GPS fix before takeoff... (satellites: " + String(getSatelliteCount()) + ")");
                        lastGpsLog = millis();
                    }
                    // Don't proceed without GPS - just wait
                }
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
            static unsigned long lastAltLog = 0;
            unsigned long timeInState = millis() - stateStartTime;
            
            // Log altitude every 2 seconds to avoid spam
            if (millis() - lastAltLog >= 2000) {
                LogProxy::log("Waiting for takeoff completion... Current altitude: " + String(current_alt, 1) + "m, Target: " + String(TAKEOFF_ALTITUDE * 0.8, 1) + "m, Time: " + String(timeInState / 1000) + "s");
                lastAltLog = millis();
            }
            
            // Wait at least 20 seconds before proceeding to loiter mode
            if (timeInState >= 20000) { // 20 second minimum wait
                if (current_alt >= TAKEOFF_ALTITUDE * 0.8) { // 80% of target altitude
                    LogProxy::log("✓ Takeoff completed! Altitude: " + String(current_alt, 1) + "m after " + String(timeInState / 1000) + "s");
                } else {
                    LogProxy::log("⚠ Proceeding to loiter mode after minimum 20s wait (altitude: " + String(current_alt, 1) + "m)");
                }
                currentState = LOITER_MODE;
                stateStartTime = millis();
                LogProxy::log("Step 4: Starting loiter mode...");
            } else if (millis() - stateStartTime >= 60000) { // 60 second maximum timeout for takeoff
                LogProxy::log("⚠ Takeoff timeout - proceeding to loiter mode anyway");
                currentState = LOITER_MODE;
                stateStartTime = millis();
            }
            break;
        }
        case LOITER_MODE: {
            // In loiter mode, the drone will automatically loiter around the current position
            // We can periodically log the status
            static unsigned long lastStatusLog = 0;
            if (millis() - lastStatusLog >= 5000) { // Log every 5 seconds
                float current_alt = getAltitude();
                LogProxy::log("Loitering... Altitude: " + String(current_alt, 1) + "m");
                lastStatusLog = millis();
            }
            break;
        }
        case COMPLETE: {
            // Mission is complete, nothing more to do
            break;
        }
    }
}

void LoiterMission::stop() {
    LogProxy::log("LoiterMission: Stopping mission");
    
    currentState = COMPLETE;
    LogProxy::log("Mission stopped and drone disarmed");
}

void LoiterMission::onCommandAck(uint16_t command, uint8_t result) {
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

const char* LoiterMission::getCurrentStateName() const {
    switch (currentState) {
        case ARM_DRONE: return "ARM_DRONE";
        case WAIT_ARM_ACK: return "WAIT_ARM_ACK";
        case SET_GUIDED_MODE: return "SET_GUIDED_MODE";
        case WAIT_MODE_ACK: return "WAIT_MODE_ACK";
        case TAKEOFF: return "TAKEOFF";
        case WAIT_TAKEOFF_ACK: return "WAIT_TAKEOFF_ACK";
        case WAIT_TAKEOFF_COMPLETE: return "WAIT_TAKEOFF_COMPLETE";
        case LOITER_MODE: return "LOITER_MODE";
        case COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
} 