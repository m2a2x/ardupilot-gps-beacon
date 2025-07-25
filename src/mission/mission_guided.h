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

    /**
     * Execute follow-me logic: send position target with offset behind beacon
     * @param offset_meters Distance behind beacon in meters (positive = behind)
     * @param altitude_offset_meters Altitude offset in meters (positive = above)
     * @param gps_valid Whether GPS has valid fix
     * @param log_prefix Optional prefix for logging messages
     * @return true if position was sent successfully, false otherwise
     */
    bool executeFollowMeLogic(float offset_meters = 3.0, float altitude_offset_meters = 0.0, bool gps_valid = true, const String& log_prefix = ""); 
}; 