#pragma once
#include "mission.h"

class LoiterMission : public Mission {
private:
    enum MissionState {
        ARM_DRONE,
        WAIT_ARM_ACK,
        SET_GUIDED_MODE,
        WAIT_MODE_ACK,
        TAKEOFF,
        WAIT_TAKEOFF_ACK,
        WAIT_TAKEOFF_COMPLETE,
        LOITER_MODE,
        COMPLETE
    };
    
    MissionState currentState;
    unsigned long stateStartTime;
    const float TAKEOFF_ALTITUDE = 2.0f; // 2 meters
    
    // Verification flags
    bool armCommandSent;
    bool modeCommandSent;
    bool takeoffCommandSent;
    bool armAckReceived;
    bool modeAckReceived;
    bool takeoffAckReceived;
    
public:
    LoiterMission() : 
        currentState(ARM_DRONE), 
        stateStartTime(0),
        armCommandSent(false),
        modeCommandSent(false),
        takeoffCommandSent(false),
        armAckReceived(false),
        modeAckReceived(false),
        takeoffAckReceived(false) {}
    
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "Loiter"; }
    const char* getType() const override { return "Loiter"; }
    
    // Callback for command acknowledgments
    void onCommandAck(uint16_t command, uint8_t result);
    
    // Check if mission is in loiter mode
    bool isInLoiterMode() const { return currentState == LOITER_MODE; }
    
    // Override the base class method to provide current state
    const char* getCurrentStateName() const override;
}; 