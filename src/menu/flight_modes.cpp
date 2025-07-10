#include "flight_modes.h"
#include "mission/mission_guided.h"
#include "mission/mission_followme.h"
#include "mission/mission_goto.h"
#include "mission/mission_arm.h"
#include "mission/mission_loiter.h"
#include "mavlink_cmds.h"  // For send_set_mode
#include "utils.h"

// External declarations
extern Mission* currentMission;  // From utils.cpp

/**
 * Initialize flight modes system
 */
void initFlightModes() {
  // Nothing to initialize for now
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
  
  switch (mode) {
    case FLIGHT_MODE_GUIDED:
      currentMission = new GuidedMission();
      currentMission->start();
      Serial.println("Guided Mode started");
      return true;

    case FLIGHT_MODE_FOLLOW_ME:
      currentMission = new FollowMeMission();
      currentMission->start();
      Serial.println("Follow Me Mode started");
      return true;

    case FLIGHT_MODE_GO_TO:
      currentMission = new GoToMission();
      currentMission->start();
      Serial.println("Go To Mode started");
      return true;

    case FLIGHT_MODE_ARM:
      currentMission = new ArmMission();
      currentMission->start();
      Serial.println("Arm Mode started");
      return true;

    case FLIGHT_MODE_BRAKE:
      send_set_mode("BRAKE");
      Serial.println("Break Mode started");
      return true;

    default:
      Serial.println("Unknown flight mode");
      return false;
  }
}

/**
 * Start a specific flight mode
 * @param mode The flight mode to start
 * @return true if mode was successfully started, false otherwise
 */
bool startFlightMode(FlightMode mode) {
  // Stop any existing mission first
  if (currentMission) {
    currentMission->stop();
    delete currentMission;
    currentMission = nullptr;
  }
  
  switch (mode) {
    case FLIGHT_MODE_GUIDED:
      currentMission = new GuidedMission();
      currentMission->start();
      Serial.println("Guided Mode started");
      return true;

    case FLIGHT_MODE_FOLLOW_ME:
      currentMission = new FollowMeMission();
      currentMission->start();
      Serial.println("Follow Me Mode started");
      return true;

    case FLIGHT_MODE_GO_TO:
      currentMission = new GoToMission();
      currentMission->start();
      Serial.println("Go To Mode started");
      return true;

    case FLIGHT_MODE_ARM:
      currentMission = new ArmMission();
      currentMission->start();
      Serial.println("Arm Mode started");
      return true;

    case FLIGHT_MODE_BRAKE:
      send_set_mode("BRAKE");
      Serial.println("Break Mode started");
      return true;

    default:
      Serial.println("Unknown flight mode");
      return false;
  }
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
    // Start Break mode (Loiter)
    send_set_mode("BRAKE");
    Serial.println("Break Mode started");
    return true;
  }
  Serial.println("No active flight mode to stop");
  return false;
}

/**
 * Check if a specific flight mode is currently active
 * @param mode The flight mode to check
 * @return true if the mode is active, false otherwise
 */
bool isFlightModeActive(FlightMode mode) {
  if (mode == FLIGHT_MODE_BRAKE) {
    return false; // BRAKE mode is no longer tracked
  }
  
  if (!currentMission) {
    return false;
  }
  
  String activeMode = currentMission->getName();
  
  switch (mode) {
    case FLIGHT_MODE_GUIDED:
      return activeMode == "Guided Mode";
    case FLIGHT_MODE_FOLLOW_ME:
      return activeMode == "Follow Me";
    case FLIGHT_MODE_GO_TO:
      return activeMode == "GoTo";
    case FLIGHT_MODE_ARM:
      return activeMode == "Arm";
    default:
      return false;
  }
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
  lines.push_back("  Guided Mode");
  lines.push_back("  Follow Me");
  lines.push_back("  Go To");
  lines.push_back("  Arm");
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
  
  // Add selection indicators and check for active mode
  String activeMode = getActiveFlightMode();
  
  // Guided Mode
  String guidedLine = (selectedOption == 0 ? "> " : "  ") + String("Guided Mode");
  lines.push_back(guidedLine);
  if (activeMode == "Guided Mode") {
    highlightLines.push_back(1);
  }
  
  // Follow Me
  String followMeLine = (selectedOption == 1 ? "> " : "  ") + String("Follow Me");
  lines.push_back(followMeLine);
  if (activeMode == "Follow Me") {
    highlightLines.push_back(2);
  }
  
  // Go To
  String goToLine = (selectedOption == 2 ? "> " : "  ") + String("Go To");
  lines.push_back(goToLine);
  if (activeMode == "GoTo") {
    highlightLines.push_back(3);
  }
  
  // Arm
  String armLine = (selectedOption == 3 ? "> " : "  ") + String("Arm");
  lines.push_back(armLine);
  if (activeMode == "Arm") {
    highlightLines.push_back(4);
  }
  
  // Back
  String backLine = (selectedOption == 4 ? "> " : "  ") + String("Back");
  lines.push_back(backLine);
} 