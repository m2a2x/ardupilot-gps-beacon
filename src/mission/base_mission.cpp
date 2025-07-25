#include "mission.h"
#include "menu/menu.h"
#include "menu/menu_types.h"  // For MenuOption enum
#include "log_proxy.h"
#include "flight_validator.h"
#include "drone_status.h"

// External declarations
extern DroneStatus droneStatus;

std::vector<MenuOption> BaseMission::getMenuOptions() const {
    return {START_MODE, STOP_MODE, RTL_MODE, BACK_TO_MODE};
}

void BaseMission::handleMenuAction(MenuOption option) {
    switch (option) {
        case START_MODE: 
            onStart();
            start(); // Call the original start method
            isRunning = true;
            break;
            
        case STOP_MODE:
            onStop();
            stop(); // Call the original stop method
            isRunning = false;
            break;
            
        case RTL_MODE:
            onRTL();
            isRunning = false;
            break;
            
        case BACK_TO_MODE:
            // This will be handled by the main menu system
            break;
            
        default:
            break;
    }
}

void BaseMission::getMenuDisplay(std::vector<String>& lines, MenuOption selectedOption) const {
    lines.push_back("== " + String(getName())+ String(getUpdateCount()) + " ==");
    
    if (isRunning) {
        lines.push_back("State: " + String(getCurrentStateName()));
    }
    
    lines.push_back("");
    lines.push_back((selectedOption == START_MODE ? "> " : "  ") + String("Start"));
    lines.push_back((selectedOption == STOP_MODE ? "> " : "  ") + String("Stop"));
    lines.push_back((selectedOption == RTL_MODE ? "> " : "  ") + String("RTL"));
    lines.push_back((selectedOption == BACK_TO_MODE ? "> " : "  ") + String("Back"));
}

bool BaseMission::checkRadioConnectivity() {
    // Check radio connectivity using flight validator
    FlightValidator::ValidationError radioError = FlightValidator::checkRadioConnectivity(droneStatus.last_heartbeat, DroneStatus::HEARTBEAT_TIMEOUT_MS);
    
    // Log when radio connectivity is restored (but only once)
    static bool wasRadioLost = false;
    
    if (radioError != FlightValidator::NO_ERROR) {
        if (!wasRadioLost) {
            wasRadioLost = true;
        }
        return false;
    } else {
        if (wasRadioLost) {
            wasRadioLost = false;
        }
    }
    
    return true;
}

const char* BaseMission::getCurrentStateName() const {
    static char stateBuffer[128];
    
    // Get base state name
    const char* baseState = isRunning ? "RUNNING" : "STOPPED";
    
    // Check radio connectivity first (highest priority)
    FlightValidator::ValidationError radioError = FlightValidator::checkRadioConnectivity(droneStatus.last_heartbeat, DroneStatus::HEARTBEAT_TIMEOUT_MS);
    if (radioError != FlightValidator::NO_ERROR) {
        snprintf(stateBuffer, sizeof(stateBuffer), "%s [RADIO: %s]", 
                baseState, FlightValidator::getErrorDescription(radioError));
        return stateBuffer;
    }
    
    return baseState;
} 