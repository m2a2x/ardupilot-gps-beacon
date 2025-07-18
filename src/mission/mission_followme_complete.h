#pragma once
#include "mission.h"
#include "flight_validator.h"

class FollowMeCompleteMission : public Mission {
private:
    enum MissionState {
        SET_GUIDED_MODE,
        ARM_DRONE,
        TAKEOFF,
        WAIT_TAKEOFF_COMPLETE,
        FOLLOW_MODE,
        COMPLETE
    };
    
    MissionState currentState;
    unsigned long stateStartTime;
    unsigned long lastPositionSend;
    const unsigned long POSITION_SEND_INTERVAL = 1000; // 1 second interval
    const float TAKEOFF_ALTITUDE = 3.0f; // 3 meters
    const double FOLLOW_OFFSET = 3.0; // 3 meters behind
    
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
    
public:
    FollowMeCompleteMission() : 
        currentState(SET_GUIDED_MODE), 
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
        beaconGpsValid(false) {}
    
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "Auto"; }
    const char* getType() const override { return "FollowMeComplete"; }
    
    // Callback for command acknowledgments
    void onCommandAck(uint16_t command, uint8_t result);
    
    // Callback for system status messages
    void onSystemStatus(uint32_t onboard_control_sensors_present, 
                       uint32_t onboard_control_sensors_enabled,
                       uint32_t onboard_control_sensors_health);
    
    // Callback for GPS status from drone
    void onGPSStatus(uint8_t fix_type);
    
    // Check if mission is in follow mode
    bool isInFollowMode() const { return currentState == FOLLOW_MODE; }
    
    // Override the base class method to provide current state
    const char* getCurrentStateName() const override;
    
    // Menu control methods - implement required pure virtual functions
    std::vector<MenuOption> getMenuOptions() const override;
    void handleMenuAction(MenuOption option) override;
    void getMenuDisplay(std::vector<String>& lines, MenuOption selectedOption) const override;
    bool isMenuActive() const override;
}; 