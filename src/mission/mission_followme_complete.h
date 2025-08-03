#pragma once
#include "mission.h"
#include "flight_validator.h"
#include "../gps_utils.h"  // For GPS calculation functions

class FollowMeCompleteMission : public Mission {
protected:
    enum MissionState {
        SET_GUIDED_MODE,
        ARM_DRONE,
        TAKEOFF,
        WAIT_TAKEOFF_COMPLETE,
        IN_FOLLOW_MODE,
        COMPLETE
    };
    
    MissionState currentState;
    unsigned long stateStartTime;
    unsigned long lastPositionSend;
    const float TAKEOFF_ALTITUDE = 6.0f; // 6 meters
    const int RETRY_TIME = 5000; // 5 seconds
    const int MAX_TAKEOFF_TIME = 60000; // 60 seconds
    const int DELAY_BEFORE_GOTO_MODE = 10000; // 10 seconds

    // Verification flags
    bool armCommandSent;
    bool modeCommandSent;
    bool takeoffCommandSent;
    bool armAckReceived;
    bool modeAckReceived;
    bool takeoffAckReceived;
    
    // Validation error tracking
    FlightValidator::ValidationError currentError;
    unsigned long errorStartTime;
    
    // Drone GPS status tracking (informational)
    uint8_t droneGpsFixType;
    
    // Beacon GPS status tracking (mission critical)
    bool beaconGpsValid;
    
    // Target position for goto mode
    double targetLat;
    double targetLon;
    float targetAlt;
    bool targetSet;
    
    // ROI control settings
    bool roiControlEnabled;
    

    
public:
    FollowMeCompleteMission() : 
        currentState(COMPLETE), 
        stateStartTime(0), 
        lastPositionSend(0),
        armCommandSent(false),
        modeCommandSent(false),
        takeoffCommandSent(false),
        armAckReceived(false),
        modeAckReceived(false),
        takeoffAckReceived(false),
        currentError(FlightValidator::NO_ERROR),
        errorStartTime(0),
        droneGpsFixType(GPS_FIX_TYPE_NO_GPS),
        beaconGpsValid(false),
        targetLat(0.0),
        targetLon(0.0),
        targetAlt(0.0f),
        targetSet(false),
        roiControlEnabled(false) {}
    
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "Auto"; }
    const char* getType() const override { return "FollowMeComplete"; }
    
    // Callback for command acknowledgments
    void onCommandAck(uint16_t command, uint8_t result) override;

    // Callback for GPS status from drone
    void onGPSStatus(uint8_t fix_type);
    
    // Check if mission is in goto mode
    bool isInGotoMode() const { return currentState == IN_FOLLOW_MODE; }
    
    // Override the base class method to provide current state
    const char* getCurrentStateName() const override;
    
    // Menu control methods - implement required pure virtual functions
    std::vector<MenuOption> getMenuOptions() const override;
    void handleMenuAction(MenuOption option) override;
    void getMenuDisplay(std::vector<String>& lines, MenuOption selectedOption) const override;
    bool isMenuActive() const override;
    
    // Method to set new target position with specific altitude
    void setNewTargetWithAltitude(float altitude);
    
    // Helper method to check if drone is in the air using multiple indicators
    bool isDroneInAir() const;
    
protected:
    // Virtual functions that can be overridden by derived classes
    virtual void handleSetGuidedMode();
    virtual void handleInFollowMode();
    
    // Virtual function to get position send interval (can be overridden)
    virtual unsigned long getPositionSendInterval() const { return 1000; } // 1 second default
}; 