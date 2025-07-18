#pragma once
#include "mission.h"

class GuidedMission : public BaseMission {
public:
    void start() override;
    void update() override;
    void stop() override;
    const char* getName() const override { return "Guided Mode"; }
    const char* getType() const override { return "Guided"; }
    
protected:
    void onStart() override;
    void onStop() override;
    void onRTL() override;
}; 