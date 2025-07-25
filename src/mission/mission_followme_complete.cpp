#include "mission_followme_complete.h"
#include <Arduino.h>
#include "mavlink_cmds.h"
#include "gps.h"
#include "conf.h"
#include "log_proxy.h"  // For logging
#include "utils.h"
#include "flight_validator.h"
#include "../menu/menu_types.h"  // For MenuOption enum
#include "../gps_utils.h"  // For GPS calculation functions
#include "../drone_status.h"  // For DroneStatus object

// External declarations
extern DroneStatus droneStatus;

void FollowMeCompleteMission::start() {
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
    
    // Reset target position
    targetLat = 0.0;
    targetLon = 0.0;
    targetAlt = 0.0f;
    targetSet = false;
    
    resetUpdateCount();
    
    // Request status text messages to get better feedback
    request_status_text(1000); // Request status text every 1 second
    
    // Request altitude data stream to ensure we receive drone altitude updates
    request_data_stream(MAV_DATA_STREAM_POSITION, 10); // 10Hz position data (includes altitude)
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
            LogProxy::log("Beacon GPS error resolved");
            currentError = FlightValidator::NO_ERROR;
            errorStartTime = 0;
        }
    }

    
    switch (currentState) {
        case SET_GUIDED_MODE: {
            handleSetGuidedMode();
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
                LogProxy::log("Arm command acknowledged successfully");
                currentState = TAKEOFF;
                stateStartTime = millis();
            } else if (millis() - stateStartTime >= RETRY_TIME) { // 5 second timeout
                LogProxy::log("Arm command timeout, retrying...");
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
                LogProxy::log("Takeoff command acknowledged");
                currentState = WAIT_TAKEOFF_COMPLETE;
                stateStartTime = millis();
            } else if (millis() - stateStartTime >= RETRY_TIME) { // 5 second timeout
                LogProxy::log("⚠ Takeoff command timeout, retrying...");
                takeoffCommandSent = false; // Retry
            }
            break;
        }
        case WAIT_TAKEOFF_COMPLETE: {
            unsigned long timeInState = millis() - stateStartTime;
            
            // Use drone altitude from DroneStatus object for takeoff detection
            float current_alt = 0.0f;
            bool altitude_available = false;
            
            if (droneStatus.isAltitudeValid() && droneStatus.altitude_relative > TAKEOFF_ALTITUDE * 0.8) {
                current_alt = droneStatus.altitude_relative;  // Use relative altitude (above home)
                altitude_available = true;
                LogProxy::log("Altitude available: " + String(current_alt) + "m");
            }
            
            // Simple timeout-based approach
            if (timeInState >= DELAY_BEFORE_GOTO_MODE || altitude_available) {
                currentState = IN_FOLLOW_MODE;
                stateStartTime = millis();
                setNewTargetWithAltitude(TAKEOFF_ALTITUDE);
            }
            break;
        }
        case IN_FOLLOW_MODE: {
            handleInFollowMode();
            break;
        }
        case COMPLETE: {
            // Mission is complete, nothing more to do
            break;
        }
    }
}

void FollowMeCompleteMission::stop() {
    // Set drone to LOITER mode for safe hovering
    send_set_mode_command("LOITER");
    currentState = COMPLETE;
}

void FollowMeCompleteMission::setNewTargetWithAltitude(float altitude) {
    if (gpsHasFix()) {
        // Get current beacon position
        double beaconLat = getLatitude();
        double beaconLon = getLongitude();
        
        // Calculate offset position 3 meters east (π/2 radians = 90 degrees = East)
        double offsetLat, offsetLon;
        calculateOffsetPosition(beaconLat, beaconLon, 3.0, M_PI / 2.0, offsetLat, offsetLon);
        
        // Set target to offset position with new altitude
        targetLat = offsetLat;
        targetLon = offsetLon;
        targetAlt = altitude;
        targetSet = true;
    } else {
        LogProxy::log("⚠ No GPS fix available, cannot set target");
        targetSet = false;
    }
}

void FollowMeCompleteMission::onCommandAck(uint16_t command, uint8_t result) {
    // Use validation component to check for errors
    FlightValidator::ValidationError error = FlightValidator::checkCommandAckError(command, result);
    
    if (error != FlightValidator::NO_ERROR) {
        currentError = error;
        errorStartTime = millis();
        LogProxy::log("⚠ " + String(FlightValidator::getErrorDescription(error)));
    } else {
        // Clear error if command was successful
        if (currentError != FlightValidator::NO_ERROR) {
            LogProxy::log("Error resolved: " + String(FlightValidator::getErrorDescription(currentError)));
            currentError = FlightValidator::NO_ERROR;
            errorStartTime = 0;
        }
    }
    
    switch (command) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (result == MAV_RESULT_ACCEPTED) {
                armAckReceived = true;
            }
            break;
            
        case MAV_CMD_DO_SET_MODE:
            if (result == MAV_RESULT_ACCEPTED) {
                modeAckReceived = true;
            }
            break;
            
        case MAV_CMD_NAV_TAKEOFF:
            if (result == MAV_RESULT_ACCEPTED) {
                takeoffAckReceived = true;
            }
            break;
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
        case IN_FOLLOW_MODE: baseState = "FOLLOW_ME"; break;
        case COMPLETE: baseState = "COMPLETE"; break;
        default: baseState = "UNKNOWN"; break;
    }
    
    // Check radio connectivity first (highest priority)
    FlightValidator::ValidationError radioError = FlightValidator::checkRadioConnectivity(droneStatus.last_heartbeat, DroneStatus::HEARTBEAT_TIMEOUT_MS);
    if (radioError != FlightValidator::NO_ERROR) {
        snprintf(stateBuffer, sizeof(stateBuffer), "%s [RADIO: %s]", 
                baseState, FlightValidator::getErrorDescription(radioError));
        return stateBuffer;
    }
    
    // If there's another error, append it to the state name
    if (currentError != FlightValidator::NO_ERROR) {
        snprintf(stateBuffer, sizeof(stateBuffer), "%s [ERROR: %s]", 
                baseState, FlightValidator::getErrorDescription(currentError));
        return stateBuffer;
    }
    
    return baseState;
}

// Menu control methods implementation
std::vector<MenuOption> FollowMeCompleteMission::getMenuOptions() const {
    std::vector<MenuOption> options;
    options.push_back(START_MODE);
    options.push_back(STOP_MODE);
    options.push_back(RTL_MODE);
    options.push_back(ALT_3M);
    options.push_back(ALT_6M);
    options.push_back(ALT_8M);
    options.push_back(BACK_TO_MODE);
    return options;
}

void FollowMeCompleteMission::handleMenuAction(MenuOption option) {
    switch (option) {
        case START_MODE:
            stop(); // Stop the mission
            start(); // Restart the mission
            break;
            
        case STOP_MODE:
            stop(); // Stop the mission
            break;
            
        case RTL_MODE:
            // Set drone to RTL mode for return to launch
            send_set_mode_command("RTL");
            LogProxy::log("FollowMeCompleteMission: Setting drone to RTL mode for return to launch");
            currentState = COMPLETE; // Complete the mission
            break;
            

            
        case ALT_3M:
            setNewTargetWithAltitude(3.0f);
            break;
            
        case ALT_6M:
            setNewTargetWithAltitude(6.0f);
            break;
            
        case ALT_8M:
            setNewTargetWithAltitude(8.0f);
            break;
            
        case BACK_TO_MODE:
            // This will be handled by the main menu system
            break;
            
        default:
            break;
    }
}

void FollowMeCompleteMission::getMenuDisplay(std::vector<String>& lines, MenuOption selectedOption) const {    
    if (currentState != COMPLETE) {
        lines.push_back(String(getCurrentStateName()));
    }
    
    lines.push_back((selectedOption == START_MODE ? "> " : "  ") + String("Start"));
    lines.push_back((selectedOption == STOP_MODE ? "> " : "  ") + String("Stop"));
    lines.push_back((selectedOption == RTL_MODE ? "> " : "  ") + String("RTL"));
    
    // Altitude selection in one line
    String altLine = "Alt: ";
    altLine += (selectedOption == ALT_3M ? ">" : " ") + String("3m ");
    altLine += (selectedOption == ALT_6M ? ">" : " ") + String("6m ");
    altLine += (selectedOption == ALT_8M ? ">" : " ") + String("8m");
    lines.push_back(altLine);
    
    lines.push_back((selectedOption == BACK_TO_MODE ? "> " : "  ") + String("Back"));
}

bool FollowMeCompleteMission::isMenuActive() const {
    return currentState != COMPLETE;
}

void FollowMeCompleteMission::handleSetGuidedMode() {
    if (!modeCommandSent) {
        send_set_mode_command("GUIDED");
        modeCommandSent = true;
        stateStartTime = millis();
    }
    // Wait for mode acknowledgment or timeout
    if (modeAckReceived) {
        LogProxy::log("GUIDED mode set successfully");
        currentState = ARM_DRONE;
        stateStartTime = millis();
    } else if (millis() - stateStartTime >= RETRY_TIME) { // 5 second timeout
        LogProxy::log("⚠ Mode command timeout, retrying...");
        modeCommandSent = false; // Retry
    }
}

void FollowMeCompleteMission::handleInFollowMode() {
    // Check if it's time to send position target and GPS has valid fix
    if (millis() - lastPositionSend >= getPositionSendInterval() && gpsHasFix() && targetSet) {
        // Send the target coordinates to the drone
        send_position_target(targetLat, targetLon, targetAlt);
        updateCount++;  // Increment counter for successful update
        lastPositionSend = millis();
    }
} 