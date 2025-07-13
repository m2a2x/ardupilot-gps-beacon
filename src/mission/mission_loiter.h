#pragma once
#include "mission.h"

class LoiterMission : public Mission {
private:
    enum MissionState {
        SET_MODE,
        WAIT_AFTER_MODE,
        ARM,
        WAIT_AFTER_ARM,
        TAKEOFF,
        LOITER
    };
    
    MissionState currentState;
    unsigned long stateStartTime;
    float takeoff_alt;
    
public:
    LoiterMission(float altitude = 10.0f) : currentState(SET_MODE), stateStartTime(0), takeoff_alt(altitude) {}
    
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "Loiter"; }
    const char* getType() const override { return "Loiter"; }
    
private:
    void executeTakeoff();
}; 