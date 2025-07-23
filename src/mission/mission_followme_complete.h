#pragma once
#include "mission.h"
#include "flight_validator.h"
#include "../gps_utils.h"  // For GPS calculation functions

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
    const float TAKEOFF_ALTITUDE = 7.0f; // 7 meters
    const double FOLLOW_OFFSET = 7.0; // 7 meters behind
    const int LOG_TIME = 2000; // 2 seconds
    const int RETRY_TIME = 5000; // 5 seconds
    const int MAX_TAKEOFF_TIME = 60000; // 60 seconds
    const int DELAY_BEFORE_FOLLOW_MODE = 20000; // 20 seconds

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
    
    // Position tracking for improved follow logic
    double lastBeaconLat;
    double lastBeaconLon;
    bool hasLastBeaconPosition;
    const float POSITION_CHANGE_THRESHOLD = 2.0f; // 2 meters threshold
    unsigned long lastYawSend;
    const unsigned long YAW_SEND_INTERVAL = 500; // 500ms interval for yaw updates
    
    // Yaw control for smooth video recording
    float lastYawRad;
    bool hasLastYaw;
    const float YAW_CHANGE_THRESHOLD = 0.174533f; // 10 degrees in radians (10 * PI / 180)
    
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
        lastBeaconLat(0.0),
        lastBeaconLon(0.0),
        hasLastBeaconPosition(false),
        lastYawSend(0),
        lastYawRad(0.0f),
        hasLastYaw(false) {}
    
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
    
private:
    // Helper method for smooth yaw control
    void updateYawWithThreshold(double beaconLat, double beaconLon);
}; 