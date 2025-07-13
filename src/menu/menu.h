#pragma once
#include <Arduino.h>
#include <vector>
#include "mission/mission.h"
#include "utils.h"

/**
 * Menu screen types for multi-level navigation
 */
enum MenuScreen {
  MAIN_MENU,      ///< Main menu with primary options
  GPS_INFO,       ///< GPS information screen
  MAVLINK_MESSAGES, ///< MAVLink messages screen (basic view)
  MAVLINK_DETAILS, ///< MAVLink messages screen (detailed view with fields)
  FLIGHT_MODES,   ///< Flight mode selection sub-menu
  FLIGHT_MODE_STATUS, ///< Individual flight mode status screen
  SETTINGS,       ///< Settings sub-menu
  GPS_MENU,       ///< GPS control sub-menu
  MISSION_STATUS, ///< Mission status screen
  GUIDED_MODE_CONTROL, ///< Guided mode control screen (start/stop/back)
  FOLLOW_ME_CONTROL,   ///< Follow me mode control screen (start/stop/back)
  AUTO_CONTROL, ///< Auto mode control screen (start/stop/back)
  GO_TO_CONTROL,       ///< Go to mode control screen (start/stop/back)
  ARM_CONTROL          ///< Arm mode control screen (start/stop/back)
};

/**
 * Available menu options for drone control
 * Each option represents a different control function
 */
enum MenuOption {
  FLIGHT_MODES_MENU, ///< Enter flight modes sub-menu
  GPS_INFO_SCREEN,   ///< Enter GPS info screen
  MAVLINK_MESSAGES_SCREEN, ///< Enter MAVLink messages screen
  MAVLINK_DETAILS_SCREEN, ///< Enter detailed MAVLink messages screen
  SETTINGS_MENU,     ///< Enter settings sub-menu
  RESTART,        ///< Restart the ESP32
  EXIT_MENU,      ///< Exit menu system and return to normal operation
  
  // Flight modes sub-menu options
  GUIDED_MODE,    ///< Enter guided mode control
  FOLLOW_ME,      ///< Enter follow me mode control
  AUTO, ///< Enter auto mode control
  GO_TO,          ///< Enter go to mode control
  ARM,            ///< Enter arm mode control
  BACK,           ///< Go back to previous menu
  
  // Flight mode status screen options
  BACK_TO_FLIGHT_MODES, ///< Go back to flight modes menu
  
  // Individual flight mode control options
  START_MODE,     ///< Start the current flight mode
  STOP_MODE,      ///< Stop the current flight mode
  BACK_TO_MODE    ///< Go back to flight modes menu
};

/**
 * Menu navigation structure
 */
struct MenuState {
  MenuScreen currentScreen;
  MenuOption currentOption;
  bool menuActive;
  int optionCount;  // Number of options in current screen
};

/**
 * Enter the menu system
 * Initializes menu state and sets first option as active
 */
void enterMenu();

/**
 * Check if currently in menu
 * @return true if menu is active, false otherwise
 */
bool isInMenu();

/**
 * Get current menu screen
 * @return current MenuScreen
 */
MenuScreen getCurrentScreen();

/**
 * Navigate to a specific menu screen
 * @param screen The screen to navigate to
 */
void navigateToScreen(MenuScreen screen);

/**
 * Go back to previous screen or exit menu
 */
void goBack();

/**
 * Select the current menu option
 * Executes the action associated with the currently selected menu option
 */
void selectMenuOption();

/**
 * Move to the next menu option
 * Cycles through available menu options in sequence
 */
void nextMenuOption();

/**
 * Move to the previous menu option
 * Cycles through available menu options in reverse sequence
 */
void previousMenuOption();

/**
 * Get the current menu display lines
 * Generates formatted text for menu display
 * @param lines Vector to store menu display lines
 */
void getMenuDisplay(std::vector<String> &lines);

/**
 * Get the current menu display lines with highlight information
 * Generates formatted text for menu display and returns lines to highlight
 * @param lines Vector to store menu display lines
 * @param highlightLines Vector to store line indices to highlight (0-based)
 */
void getMenuDisplayWithHighlight(std::vector<String> &lines, std::vector<int> &highlightLines); 