#include "mission_guided.h"
#include "mavlink_cmds.h"
#include "gps.h"
#include "utils.h"
#include "log_proxy.h"

void GuidedMission::start() {
    // This is now handled by BaseMission::handleMenuAction
    // The actual start logic is in onStart()
}

void GuidedMission::update() {
    if (isRunning) {
        static unsigned long lastPositionSend = 0;
        const unsigned long POSITION_SEND_INTERVAL = 1000; // 1 second interval
        
        // Check if it's time to send position target and GPS has valid fix
        if (millis() - lastPositionSend >= POSITION_SEND_INTERVAL && gpsHasFix()) {
            if (executeFollowMeLogic(3.0, 0.0, true, "Guided")) {
                updateCount++;  // Increment counter for successful update
                lastPositionSend = millis();
            }
        }
    }
}

void GuidedMission::stop() {
    // This is now handled by BaseMission::handleMenuAction
    // The actual stop logic is in onStop()
}

void GuidedMission::onStart() {
    // Mission-specific start logic
    send_set_mode("GUIDED");
    resetUpdateCount();  // Reset counter when starting
    LogProxy::log("Starting Guided Mode...");
}

void GuidedMission::onStop() {
    // Mission-specific stop logic
    // Set drone to LOITER mode for safe hovering
    send_set_mode_command("LOITER");
    LogProxy::log("Setting drone to LOITER mode for safe hovering");
}

void GuidedMission::onRTL() {
    // Mission-specific RTL logic
    // Set drone to RTL mode for return to launch
    send_set_mode_command("RTL");
    LogProxy::log("Setting drone to RTL mode for return to launch");
} 