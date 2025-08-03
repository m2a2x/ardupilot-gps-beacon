#pragma once
#include "mission_followme_complete.h"

class FollowMission : public FollowMeCompleteMission {
static const uint8_t CAPABILITIES_POS = (1 << 0);
static const uint8_t CAPABILITIES_VEL = (1 << 1);
static const uint8_t CAPABILITIES_ACCEL = (1 << 2);
static const uint8_t CAPABILITIES_ATT_RATES = (1 << 3);
private:
    // Follow me specific constants
    const unsigned long FOLLOW_UPDATE_INTERVAL = 100; // 100ms for 10Hz updates like Lua script
    // const uint8_t FOLLOW_TARGET_CAPABILITIES_POS = 0x01;  // Position capability
    
    // Follow me state tracking
    unsigned long lastFollowTargetSend;
    
public:
    FollowMission() : FollowMeCompleteMission(), 
                      lastFollowTargetSend(0) {}
    
    const char* getName() const override { return "Follow"; }
    const char* getType() const override { return "Follow"; }
    
    // Override start method to call parent and adjust targetAlt
    void start() override;
    
protected:
    // Override the virtual functions from parent class=
    void handleInFollowMode() override;
    unsigned long getPositionSendInterval() const override { return FOLLOW_UPDATE_INTERVAL; }
    
private:
    // Follow me specific functions
    void sendFollowTargetMessage();
}; 