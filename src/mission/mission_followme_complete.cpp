#include "mission_followme_complete.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"
#include "conf.h"
#include "log_proxy.h"  // For logging
#include "utils.h"
#include "flight_validator.h"

void FollowMeCompleteMission::start() {
    LogProxy::log("AutoMission: Starting auto mission");
    currentState = SET_GUIDED_MODE;
    stateStartTime = millis();
    lastPositionSend = 0;
    
    // Reset verification flags
    armCommandSent = false;
    modeCommandSent = false;
    takeoffCommandSent = false;
    armAckReceived = false;
    modeAckReceived = false;
    takeoffAckReceived = false;
    
    // Reset validation error
    currentError = FlightValidator::NO_ERROR;
    errorStartTime = 0;
    
    // Reset GPS status
    droneGpsFixType = GPS_FIX_TYPE_NO_GPS;
    beaconGpsValid = false;
    
    resetUpdateCount();
    
    // Request status text messages to get better feedback
    request_status_text(1000); // Request status text every 1 second
    
    LogProxy::log("Mission started: Will set GUIDED mode, arm, takeoff to 10m, then follow");
}

void FollowMeCompleteMission::update() {
    // Check beacon GPS issues (mission critical)
    FlightValidator::ValidationError beaconGpsError = FlightValidator::checkBeaconGPSIssues();
    if (beaconGpsError != FlightValidator::NO_ERROR) {
        if (currentError != beaconGpsError) {
            currentError = beaconGpsError;
            errorStartTime = millis();
            LogProxy::log("⚠ Beacon GPS: " + String(FlightValidator::getErrorDescription(beaconGpsError)));
        }
        beaconGpsValid = false;
        return; // Don't proceed without beacon GPS
    } else {
        beaconGpsValid = true;
        // Clear beacon GPS errors if resolved
        if (currentError == FlightValidator::GPS_NO_FIX) {
            LogProxy::log("✓ Beacon GPS error resolved");
            currentError = FlightValidator::NO_ERROR;
            errorStartTime = 0;
        }
    }
    
    switch (currentState) {
        case SET_GUIDED_MODE: {
            if (!modeCommandSent) {
                LogProxy::log("Step 1: Setting GUIDED mode...");
                send_set_mode_command("GUIDED");
                modeCommandSent = true;
                stateStartTime = millis();
            }
            // Wait for mode acknowledgment or timeout
            if (modeAckReceived) {
                LogProxy::log("✓ GUIDED mode set successfully");
                currentState = ARM_DRONE;
                stateStartTime = millis();
            } else if (millis() - stateStartTime >= 5000) { // 5 second timeout
                LogProxy::log("⚠ Mode command timeout, retrying...");
                modeCommandSent = false; // Retry
            }
            break;
        }
        case ARM_DRONE: {
            if (!armCommandSent) {
                LogProxy::log("Step 2: Sending arm command...");
                send_arm_command(true, true);
                armCommandSent = true;
                stateStartTime = millis();
            }
            // Wait for arm acknowledgment or timeout
            if (armAckReceived) {
                LogProxy::log("✓ Arm command acknowledged successfully");
                currentState = TAKEOFF;
                stateStartTime = millis();
            } else if (millis() - stateStartTime >= 5000) { // 5 second timeout
                LogProxy::log("⚠ Arm command timeout, retrying...");
                armCommandSent = false; // Retry
            }
            break;
        }
        case TAKEOFF: {
            if (!takeoffCommandSent) {
                // Check if beacon has GPS fix before sending takeoff command (mission critical)
                if (beaconGpsValid) {
                    LogProxy::log("Step 3: Sending takeoff command to " + String(TAKEOFF_ALTITUDE, 1) + "m... (Beacon GPS OK)");
                    send_takeoff_command(TAKEOFF_ALTITUDE);
                    takeoffCommandSent = true;
                    stateStartTime = millis();
                } else {
                    static unsigned long lastGpsLog = 0;
                    if (millis() - lastGpsLog >= 2000) { // Log every 2 seconds to avoid spam
                        LogProxy::log("⚠ Waiting for beacon GPS fix before takeoff... (beacon: " + String(beaconGpsValid ? "OK" : "NO FIX") + ", drone: " + String(droneGpsFixType) + ")");
                        lastGpsLog = millis();
                    }
                    // Don't proceed without beacon GPS - just wait
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
            
            // Wait at least 20 seconds before proceeding to follow mode
            if (timeInState >= 20000) { // 20 second minimum wait
                if (current_alt >= TAKEOFF_ALTITUDE * 0.8) { // 80% of target altitude
                    LogProxy::log("✓ Takeoff completed! Altitude: " + String(current_alt, 1) + "m after " + String(timeInState / 1000) + "s");
                } else {
                    LogProxy::log("⚠ Proceeding to follow mode after minimum 20s wait (altitude: " + String(current_alt, 1) + "m)");
                }
                currentState = FOLLOW_MODE;
                stateStartTime = millis();
                LogProxy::log("Step 4: Starting follow-me mode...");
            } else if (millis() - stateStartTime >= 60000) { // 60 second maximum timeout for takeoff
                LogProxy::log("⚠ Takeoff timeout - proceeding to follow mode anyway");
                currentState = FOLLOW_MODE;
                stateStartTime = millis();
            }
            break;
        }
        case FOLLOW_MODE: {
            // Check if it's time to send position target and GPS has valid fix
            if (millis() - lastPositionSend >= POSITION_SEND_INTERVAL && beaconGpsValid) {
                if (executeFollowMeLogic(3.0, true, "FollowMe")) {
                    LogProxy::log("Following... Beacon GPS OK, Drone GPS: " + String(droneGpsFixType));
                    updateCount++;  // Increment counter for successful update
                    lastPositionSend = millis();
                }
            } else if (!beaconGpsValid) {
                static unsigned long lastGpsLog = 0;
                if (millis() - lastGpsLog >= 2000) { // Log every 2 seconds to avoid spam
                    LogProxy::log("⚠ Following paused - waiting for beacon GPS fix (beacon: " + String(beaconGpsValid ? "OK" : "NO FIX") + ", drone: " + String(droneGpsFixType) + ")");
                    lastGpsLog = millis();
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
    
    currentState = COMPLETE;
    LogProxy::log("Mission stopped and drone disarmed");
}


void FollowMeCompleteMission::onCommandAck(uint16_t command, uint8_t result) {
    LogProxy::log("Command ACK received: command=" + String(command) + ", result=" + String(result));
    
    // Use validation component to check for errors
    FlightValidator::ValidationError error = FlightValidator::checkCommandAckError(command, result);
    
    if (error != FlightValidator::NO_ERROR) {
        currentError = error;
        errorStartTime = millis();
        LogProxy::log("⚠ " + String(FlightValidator::getErrorDescription(error)));
    } else {
        // Clear error if command was successful
        if (currentError != FlightValidator::NO_ERROR) {
            LogProxy::log("✓ Error resolved: " + String(FlightValidator::getErrorDescription(currentError)));
            currentError = FlightValidator::NO_ERROR;
            errorStartTime = 0;
        }
    }
    
    switch (command) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (result == MAV_RESULT_ACCEPTED) {
                armAckReceived = true;
                LogProxy::log("✓ Arm command accepted by drone");
            }
            break;
            
        case MAV_CMD_DO_SET_MODE:
            if (result == MAV_RESULT_ACCEPTED) {
                modeAckReceived = true;
                LogProxy::log("✓ Mode change accepted by drone");
            }
            break;
            
        case MAV_CMD_NAV_TAKEOFF:
            if (result == MAV_RESULT_ACCEPTED) {
                takeoffAckReceived = true;
                LogProxy::log("✓ Takeoff command accepted by drone");
            }
            break;
    }
}

void FollowMeCompleteMission::onSystemStatus(uint32_t onboard_control_sensors_present, 
                                           uint32_t onboard_control_sensors_enabled,
                                           uint32_t onboard_control_sensors_health) {
    // Use validation component to check for system issues
    FlightValidator::ValidationError error = FlightValidator::checkSystemStatus(
        onboard_control_sensors_present, 
        onboard_control_sensors_enabled, 
        onboard_control_sensors_health
    );
    
    if (error != FlightValidator::NO_ERROR) {
        if (currentError != error) {
            currentError = error;
            errorStartTime = millis();
            LogProxy::log("⚠ " + String(FlightValidator::getErrorDescription(error)));
        }
    } else if (currentError == FlightValidator::SENSOR_FAILURE || 
               currentError == FlightValidator::BATTERY_LOW || 
               currentError == FlightValidator::RC_SIGNAL_LOST) {
        // Clear system-related errors if they're resolved
        LogProxy::log("✓ System error resolved: " + String(FlightValidator::getErrorDescription(currentError)));
        currentError = FlightValidator::NO_ERROR;
        errorStartTime = 0;
    }
}

void FollowMeCompleteMission::onGPSStatus(uint8_t fix_type) {
    // Update drone GPS status
    droneGpsFixType = fix_type;
    
    // Log GPS status changes
    static uint8_t lastFixType = 0;
    
    if (fix_type != lastFixType) {
        LogProxy::log("GPS Status: fix=" + String(fix_type));
        lastFixType = fix_type;
    }
}

const char* FollowMeCompleteMission::getCurrentStateName() const {
    static char stateBuffer[128];
    
    // Get base state name
    const char* baseState;
    switch (currentState) {
        case SET_GUIDED_MODE: baseState = "SET_GUIDED_MODE"; break;
        case ARM_DRONE: baseState = "ARM_DRONE"; break;
        case TAKEOFF: baseState = "TAKEOFF"; break;
        case WAIT_TAKEOFF_COMPLETE: baseState = "WAIT_TAKEOFF_COMPLETE"; break;
        case FOLLOW_MODE: baseState = "FOLLOW_MODE"; break;
        case COMPLETE: baseState = "COMPLETE"; break;
        default: baseState = "UNKNOWN"; break;
    }
    
    // If there's an error, append it to the state name
    if (currentError != FlightValidator::NO_ERROR) {
        snprintf(stateBuffer, sizeof(stateBuffer), "%s [ERROR: %s]", 
                baseState, FlightValidator::getErrorDescription(currentError));
        return stateBuffer;
    }
    
    return baseState;
} 