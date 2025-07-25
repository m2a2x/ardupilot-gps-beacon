#pragma once
#include <Arduino.h>
#include <vector>
#include "mission/mission.h"
#include "menu_types.h"

/**
 * Flight mode types
 */
enum FlightMode {
  FLIGHT_MODE_GUIDED,
  FLIGHT_MODE_AUTO,
  FLIGHT_MODE_GO_TO,
  FLIGHT_MODE_FOLLOW
};

/**
 * Flight mode configuration structure
 */
struct FlightModeConfig {
  FlightMode mode;
  const char* displayName;
  const char* missionName;
  MenuScreen controlScreen;
  MenuOption menuOption;
  
  // Mission factory function type
  typedef Mission* (*MissionFactory)();
  MissionFactory createMission;
};

/**
 * Initialize flight modes system
 */
void initFlightModes();

/**
 * Execute a specific flight mode
 * @param mode The flight mode to execute
 * @return true if mode was successfully started, false otherwise
 */
bool executeFlightMode(FlightMode mode);

/**
 * Start a specific flight mode
 * @param mode The flight mode to start
 * @return true if mode was successfully started, false otherwise
 */
bool startFlightMode(FlightMode mode);

/**
 * Stop the currently active flight mode
 * @return true if mode was successfully stopped, false otherwise
 */
bool stopFlightMode();

/**
 * Check if a specific flight mode is currently active
 * @param mode The flight mode to check
 * @return true if the mode is active, false otherwise
 */
bool isFlightModeActive(FlightMode mode);

/**
 * Get the name of the currently active flight mode
 * @return String name of active mode, or "None" if no mode is active
 */
String getActiveFlightMode();

/**
 * Get flight mode configuration by enum
 * @param mode The flight mode enum
 * @return Pointer to flight mode configuration, or nullptr if not found
 */
const FlightModeConfig* getFlightModeConfig(FlightMode mode);

/**
 * Get flight mode configuration by menu option
 * @param menuOption The menu option
 * @return Pointer to flight mode configuration, or nullptr if not found
 */
const FlightModeConfig* getFlightModeConfigByMenuOption(MenuOption menuOption);

/**
 * Get flight mode configuration by control screen
 * @param screen The control screen
 * @return Pointer to flight mode configuration, or nullptr if not found
 */
const FlightModeConfig* getFlightModeConfigByScreen(MenuScreen screen);

/**
 * Get all flight mode configurations
 * @return Array of flight mode configurations
 */
const FlightModeConfig* getAllFlightModeConfigs();

/**
 * Get the number of flight modes
 * @return Number of flight modes
 */
int getFlightModeCount();

/**
 * Get flight modes display lines for menu
 * @param lines Vector to store the formatted flight modes lines
 */
void getFlightModesDisplay(std::vector<String> &lines);

/**
 * Get flight modes display with highlight information
 * @param lines Vector to store the formatted flight modes lines
 * @param highlightLines Vector to store line indices to highlight
 * @param selectedOption Currently selected option (0-based)
 */
void getFlightModesDisplayWithHighlight(std::vector<String> &lines, std::vector<int> &highlightLines, int selectedOption); 