#pragma once
#include "mission.h"

class GoToMission : public BaseMission {
public:
    const char* getName() const override { return "GoTo"; }
    const char* getType() const override { return "GoTo"; }
    
protected:
    void onStart() override;
    void onStop() override;
    void onRTL() override;
}; 