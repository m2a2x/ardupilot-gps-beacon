#pragma once
#include "mission.h"

class FollowMeMission : public Mission {
public:
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "FollowMe"; }
}; 