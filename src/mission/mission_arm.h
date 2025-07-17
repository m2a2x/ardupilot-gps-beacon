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
    
public:
    ArmMission() : currentState(ARM), stateStartTime(0) {}
    
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "Arm"; }
    const char* getType() const override { return "Arm"; }
    
    // Static factory methods for convenience
    static ArmMission* createArmMission() { return new ArmMission(); }
}; 