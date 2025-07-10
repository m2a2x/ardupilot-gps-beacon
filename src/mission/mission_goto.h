#pragma once
#include "mission.h"

class GoToMission : public Mission {
public:
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "GoTo"; }

private:
    double target_lat;
    double target_lon;
    float target_alt;
    bool target_set;
}; 