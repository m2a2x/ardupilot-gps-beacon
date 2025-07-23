#include "flight_modes.h"
#include "mission/mission_guided.h"
#include "mission/mission_followme_complete.h"
#include "mission/mission_goto.h"
#include "mavlink_cmds.h"  // For send_set_mode
#include "utils.h"
#include "log_proxy.h"  // For logging

// External declarations
extern Mission* currentMission;  // From utils.cpp

// Mission factory functions
static Mission* createGuidedMission() { return new GuidedMission(); }
static Mission* createFollowMeCompleteMission() { return new FollowMeCompleteMission(); }
static Mission* createGoToMission() { return new GoToMission(); }

// Centralized flight mode configuration
static const FlightModeConfig FLIGHT_MODE_CONFIGS[] = {
  {
    FLIGHT_MODE_GUIDED,
    "Guided Mode",
    "Guided Mode",
    GUIDED_MODE_CONTROL,
    GUIDED_MODE,
    createGuidedMission
  },
  {
    FLIGHT_MODE_AUTO,
    "Auto",
    "Auto",
    AUTO_CONTROL,
    AUTO,
    createFollowMeCompleteMission
  },
  {
    FLIGHT_MODE_GO_TO,
    "Go To",
    "GoTo",
    GO_TO_CONTROL,
    GO_TO,
    createGoToMission
  }
};

static const int FLIGHT_MODE_COUNT = sizeof(FLIGHT_MODE_CONFIGS) / sizeof(FLIGHT_MODE_CONFIGS[0]);

/**
 * Initialize flight modes system
 */
void initFlightModes() {
  // Nothing to initialize for now
}

/**
 * Get flight mode configuration by enum
 */
const FlightModeConfig* getFlightModeConfig(FlightMode mode) {
  for (int i = 0; i < FLIGHT_MODE_COUNT; i++) {
    if (FLIGHT_MODE_CONFIGS[i].mode == mode) {
      return &FLIGHT_MODE_CONFIGS[i];
    }
  }
  return nullptr;
}

/**
 * Get flight mode configuration by menu option
 */
const FlightModeConfig* getFlightModeConfigByMenuOption(MenuOption menuOption) {
  for (int i = 0; i < FLIGHT_MODE_COUNT; i++) {
    if (FLIGHT_MODE_CONFIGS[i].menuOption == menuOption) {
      return &FLIGHT_MODE_CONFIGS[i];
    }
  }
  return nullptr;
}

/**
 * Get flight mode configuration by control screen
 */
const FlightModeConfig* getFlightModeConfigByScreen(MenuScreen screen) {
  for (int i = 0; i < FLIGHT_MODE_COUNT; i++) {
    if (FLIGHT_MODE_CONFIGS[i].controlScreen == screen) {
      return &FLIGHT_MODE_CONFIGS[i];
    }
  }
  return nullptr;
}

/**
 * Get all flight mode configurations
 */
const FlightModeConfig* getAllFlightModeConfigs() {
  return FLIGHT_MODE_CONFIGS;
}

/**
 * Get the number of flight modes
 */
int getFlightModeCount() {
  return FLIGHT_MODE_COUNT;
}

/**
 * Execute a specific flight mode
 * @param mode The flight mode to execute
 * @return true if mode was successfully started, false otherwise
 */
bool executeFlightMode(FlightMode mode) {
  // Stop any existing mission first
  if (currentMission) {
    currentMission->stop();
    delete currentMission;
    currentMission = nullptr;
  }
  
  const FlightModeConfig* config = getFlightModeConfig(mode);
  if (!config) {
    LogProxy::log("Unknown flight mode");
    return false;
  }
  

  
  if (config->createMission) {
    currentMission = config->createMission();
    LogProxy::log(String(config->displayName) + " created - press Start to begin");
    return true;
  }
  
  LogProxy::log("Flight mode has no mission factory");
  return false;
}

/**
 * Start a specific flight mode
 * @param mode The flight mode to start
 * @return true if mode was successfully started, false otherwise
 */
bool startFlightMode(FlightMode mode) {
  return executeFlightMode(mode);
}

/**
 * Stop the currently active flight mode
 * @return true if mode was successfully stopped, false otherwise
 */
bool stopFlightMode() {
  if (currentMission) {
    currentMission->stop();
    delete currentMission;
    currentMission = nullptr;
    // Send LOITER command to stop the vehicle
    send_set_mode("LOITER");
    LogProxy::log("Flight mode stopped, vehicle in LOITER");
    return true;
  }
  LogProxy::log("No active flight mode to stop");
  return false;
}

/**
 * Check if a specific flight mode is currently active
 * @param mode The flight mode to check
 * @return true if the mode is active, false otherwise
 */
bool isFlightModeActive(FlightMode mode) {
  if (!currentMission) {
    return false;
  }
  
  const FlightModeConfig* config = getFlightModeConfig(mode);
  if (!config) {
    return false;
  }
  
  String activeMode = currentMission->getName();
  return activeMode == config->missionName;
}

/**
 * Get the name of the currently active flight mode
 * @return String name of active mode, or "None" if no mode is active
 */
String getActiveFlightMode() {
  if (currentMission) {
    return currentMission->getName();
  }
  return "None";
}

/**
 * Get flight modes display lines for menu
 * @param lines Vector to store the formatted flight modes lines
 */
void getFlightModesDisplay(std::vector<String> &lines) {
  lines.push_back("== FLIGHT MODES ==");
  
  String activeMode = getActiveFlightMode();
  
  // Add all flight modes
  for (int i = 0; i < FLIGHT_MODE_COUNT; i++) {
    const FlightModeConfig& config = FLIGHT_MODE_CONFIGS[i];
    String line = "  " + String(config.displayName);
    if (activeMode == config.missionName) {
      line += " *";
    }
    lines.push_back(line);
  }
  
  lines.push_back("  Back");
}

/**
 * Get flight modes display with highlight information
 * @param lines Vector to store the formatted flight modes lines
 * @param highlightLines Vector to store line indices to highlight
 * @param selectedOption Currently selected option (0-based)
 */
void getFlightModesDisplayWithHighlight(std::vector<String> &lines, std::vector<int> &highlightLines, int selectedOption) {
  lines.push_back("== FLIGHT MODES ==");
  
  String activeMode = getActiveFlightMode();
  
  // Add all flight modes
  for (int i = 0; i < FLIGHT_MODE_COUNT; i++) {
    const FlightModeConfig& config = FLIGHT_MODE_CONFIGS[i];
    String line = (selectedOption == i ? "> " : "  ") + String(config.displayName);
    if (activeMode == config.missionName) {
      line += " *";
      highlightLines.push_back(i + 1); // +1 because line 0 is the header
    }
    lines.push_back(line);
  }
  
  // Back option
  String backLine = (selectedOption == FLIGHT_MODE_COUNT ? "> " : "  ") + String("Back");
  lines.push_back(backLine);
} 