#pragma once
#include "mission.h"

class FollowMeMission : public Mission {
public:
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "Follow Me"; }
    const char* getType() const override { return "FollowMe"; }
}; 