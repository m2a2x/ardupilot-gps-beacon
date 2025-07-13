#pragma once
#include "mission.h"

class FollowMeCompleteMission : public Mission {
private:
    enum MissionState {
        ARM_DRONE,
        WAIT_ARM_ACK,
        SET_GUIDED_MODE,
        WAIT_MODE_ACK,
        TAKEOFF,
        WAIT_TAKEOFF_ACK,
        WAIT_TAKEOFF_COMPLETE,
        FOLLOW_MODE,
        COMPLETE
    };
    
    MissionState currentState;
    unsigned long stateStartTime;
    unsigned long lastPositionSend;
    const unsigned long POSITION_SEND_INTERVAL = 1000; // 1 second interval
    const float TAKEOFF_ALTITUDE = 8.0f; // 10 meters
    const double FOLLOW_OFFSET = 3.0; // 3 meters behind
    
    // Verification flags
    bool armCommandSent;
    bool modeCommandSent;
    bool takeoffCommandSent;
    bool armAckReceived;
    bool modeAckReceived;
    bool takeoffAckReceived;
    
public:
    FollowMeCompleteMission() : 
        currentState(ARM_DRONE), 
        stateStartTime(0), 
        lastPositionSend(0),
        armCommandSent(false),
        modeCommandSent(false),
        takeoffCommandSent(false),
        armAckReceived(false),
        modeAckReceived(false),
        takeoffAckReceived(false) {}
    
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "Auto"; }
    const char* getType() const override { return "FollowMeComplete"; }
    
    // Callback for command acknowledgments
    void onCommandAck(uint16_t command, uint8_t result);
    
    // Check if mission is in follow mode
    bool isInFollowMode() const { return currentState == FOLLOW_MODE; }
    
    // Override the base class method to provide current state
    const char* getCurrentStateName() const override;
}; 