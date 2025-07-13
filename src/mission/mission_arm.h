#pragma once
#include "mission.h"

class ArmMission : public Mission {
private:
    enum MissionState {
        ARM,
        WAIT_AFTER_ARM,
        COMPLETE
    };
    
    MissionState currentState;
    unsigned long stateStartTime;
    bool shouldArm;
    
public:
    ArmMission(bool arm = true) : currentState(ARM), stateStartTime(0), shouldArm(arm) {}
    
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return shouldArm ? "Arm" : "Disarm"; }
    const char* getType() const override { return "Arm"; }
    
    // Static factory methods for convenience
    static ArmMission* createArmMission() { return new ArmMission(true); }
    static ArmMission* createDisarmMission() { return new ArmMission(false); }
}; 