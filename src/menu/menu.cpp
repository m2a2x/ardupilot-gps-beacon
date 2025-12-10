#include "menu.h"
#include "conf.h"  // For configuration constants
#include "gps.h"   // For GPS functions
#include "mavlink_cmds.h"  // For MAVLink commands
#include "battery.h"  // For battery functions
#include "udp_module.h"  // For UDPClient
#include "mission/mission.h"  // For mission management
#include "tasks.h"  // For drone altitude functions

#include "mission/mission_guided.h"
#include "mission/mission_followme_complete.h"
#include "mission/mission_goto.h"

#include "mission/mission_menu_manager.h"  // For mission menu management
#include "utils.h"  // For utility functions
#include "udp_module.h"  // For UDP module
#include "flight_modes.h"  // For flight modes functionality
#include "log_proxy.h"  // For logging

// External declarations
extern unsigned long lastGPSUpdate;     // From gps.cpp
extern Mission* currentMission;         // From utils.cpp
extern UDPModule udpModule;             // From udp_module.cpp
extern bool gps_enabled;                // From conf.cpp
extern uint32_t followMeUpdates;        // From tasks.cpp

// Menu option arrays for each screen
const MenuOption MAIN_MENU_OPTIONS[] = {
  FLIGHT_MODES_MENU,
  GPS_INFO_SCREEN,
  SETTINGS_MENU,
  RESTART,
  EXIT_MENU
};

const MenuOption FLIGHT_MODES_OPTIONS[] = {
  GUIDED_MODE,
  AUTO,
  GO_TO,
  FOLLOW,
  BACK
};

const MenuOption FLIGHT_MODE_STATUS_OPTIONS[] = {
  BACK_TO_FLIGHT_MODES
};

const MenuOption SETTINGS_OPTIONS[] = {
  UDP_TOGGLE,
  GPS_TOGGLE,
  GPS_SIMULATION,
  MAVLINK_DETAILS_TOGGLE,
  BACK
};

const MenuOption GPS_MENU_OPTIONS[] = {
  BACK
};

const MenuOption MAVLINK_DETAILS_OPTIONS[] = {
  BACK
};

const MenuOption FLIGHT_MODE_CONTROL_OPTIONS[] = {
  START_MODE,
  STOP_MODE,
  BACK_TO_MODE
};

// Menu state variables
static MenuState menuState = {
  .currentScreen = MAIN_MENU,
  .currentOption = FLIGHT_MODES_MENU,
  .menuActive = false,
  .optionCount = 0
};

// Navigation history for back button functionality
static std::vector<MenuScreen> screenHistory;

// Follow me status (legacy - kept for compatibility)
uint32_t followMeUpdates = 0;
bool followMeEnabled = false;
static bool followMeActive = false;  // Tracks if FollowMe mode is active

// Settings state variables
static bool mavlinkDetailsVisible = false;  // Controls visibility of MAVLink details in main menu

// Previous GPS position for velocity calculation
static double lastLat = 0.0;
static double lastLon = 0.0;
static float lastAlt = 0.0;
static unsigned long lastPosTime = 0;

/**
 * Convert MenuOption enum to array index for a specific screen
 * This function dynamically finds the position of a menu option in its screen's options array,
 * making the code more maintainable and eliminating hardcoded index values.
 * 
 * @param screen The menu screen
 * @param option The MenuOption enum value
 * @return Array index for the screen's options array, or 0 if not found
 */
int getMenuOptionIndex(MenuScreen screen, MenuOption option) {
  const MenuOption* options = getMenuOptions(screen);
  int count = getMenuOptionCount(screen);
  
  if (!options || count == 0) {
    return 0;
  }
  
  // Find the option in the array and return its index
  for (int i = 0; i < count; i++) {
    if (options[i] == option) {
      return i;
    }
  }
  
  return 0; // Default to first option if not found
}

/**
 * Get current main menu options (including conditional MAVLink details)
 * @param options Array to fill with main menu options
 * @return Number of options
 */
static int getCurrentMainMenuOptions(MenuOption* options) {
  int count = 0;
  if (gps_enabled) {
    options[count++] = FLIGHT_MODES_MENU;
  }
  options[count++] = GPS_INFO_SCREEN;
  if (mavlinkDetailsVisible) {
    options[count++] = MAVLINK_DETAILS_SCREEN;
  }
  options[count++] = SETTINGS_MENU;
  options[count++] = RESTART;
  options[count++] = EXIT_MENU;
  return count;
}

/**
 * Get menu options for a specific screen
 * @param screen The screen to get options for
 * @return Array of menu options for the screen
 */
const MenuOption* getMenuOptions(MenuScreen screen) {
  switch (screen) {
    case MAIN_MENU:
      return MAIN_MENU_OPTIONS; // Note: This won't be used for navigation due to dynamic nature
    case FLIGHT_MODES:
      return FLIGHT_MODES_OPTIONS;
    case FLIGHT_MODE_STATUS:
      return FLIGHT_MODE_STATUS_OPTIONS;
    case SETTINGS:
      return SETTINGS_OPTIONS;
    case GPS_MENU:
      return GPS_MENU_OPTIONS;
    case MAVLINK_DETAILS:
      return MAVLINK_DETAILS_OPTIONS;
    case GUIDED_MODE_CONTROL:
    case AUTO_CONTROL:
    case GO_TO_CONTROL:
    case FOLLOW_CONTROL:
      // These screens now delegate to the mission menu manager
      // The actual options will be determined by the current mission
      return nullptr;
    default:
      return nullptr;
  }
}

/**
 * Get the number of options for a specific screen
 * Uses sizeof() to dynamically calculate array length, eliminating the need
 * to maintain separate count variables and ensuring they stay in sync.
 * 
 * @param screen The screen to get option count for
 * @return Number of options in the screen
 */
int getMenuOptionCount(MenuScreen screen) {
  switch (screen) {
    case MAIN_MENU: {
      // Calculate dynamic main menu count
      MenuOption options[6];
      return getCurrentMainMenuOptions(options);
    }
    case FLIGHT_MODES:
      return sizeof(FLIGHT_MODES_OPTIONS) / sizeof(FLIGHT_MODES_OPTIONS[0]);
    case FLIGHT_MODE_STATUS:
      return sizeof(FLIGHT_MODE_STATUS_OPTIONS) / sizeof(FLIGHT_MODE_STATUS_OPTIONS[0]);
    case SETTINGS:
      return sizeof(SETTINGS_OPTIONS) / sizeof(SETTINGS_OPTIONS[0]);
    case GPS_MENU:
      return sizeof(GPS_MENU_OPTIONS) / sizeof(GPS_MENU_OPTIONS[0]);
    case MAVLINK_DETAILS:
      return sizeof(MAVLINK_DETAILS_OPTIONS) / sizeof(MAVLINK_DETAILS_OPTIONS[0]);
    case GUIDED_MODE_CONTROL:
    case AUTO_CONTROL:
    case GO_TO_CONTROL:
    case FOLLOW_CONTROL:
      // These screens now delegate to the mission menu manager
      if (currentMission) {
        auto options = MissionMenuManager::getInstance().getMissionMenuOptions(currentMission);
        return options.size();
      }
      return 0;
    default:
      return 1; // Info screens have no options, just display
  }
}

/**
 * Get the next option in sequence for a screen
 * @param screen The current screen
 * @param currentOption The current option
 * @return The next option in sequence
 */
MenuOption getNextMenuOption(MenuScreen screen, MenuOption currentOption) {
  // Handle main menu dynamically
  if (screen == MAIN_MENU) {
    MenuOption mainMenuOptions[6]; // Max possible options
    int count = getCurrentMainMenuOptions(mainMenuOptions);
    
    // Find current option in array
    for (int i = 0; i < count; i++) {
      if (mainMenuOptions[i] == currentOption) {
        // Return next option (wrap around)
        return mainMenuOptions[(i + 1) % count];
      }
    }
    // If not found, return first option
    return mainMenuOptions[0];
  }

  // Handle mission-specific screens
  if (screen == GUIDED_MODE_CONTROL ||
      screen == AUTO_CONTROL ||
      screen == GO_TO_CONTROL ||
      screen == FOLLOW_CONTROL) {
    if (currentMission) {
      return MissionMenuManager::getInstance().getNextMenuOption(currentMission, currentOption);
    }
    return currentOption;
  }
  
  // Handle regular screens
  const MenuOption* options = getMenuOptions(screen);
  int count = getMenuOptionCount(screen);
  
  if (!options || count == 0) {
    return currentOption; // Return current if no options
  }
  
  // Find current option in array
  for (int i = 0; i < count; i++) {
    if (options[i] == currentOption) {
      // Return next option (wrap around)
      return options[(i + 1) % count];
    }
  }
  
  // If not found, return first option
  return options[0];
}

/**
 * Get the previous option in sequence for a screen
 * @param screen The current screen
 * @param currentOption The current option
 * @return The previous option in sequence
 */
MenuOption getPreviousMenuOption(MenuScreen screen, MenuOption currentOption) {
  // Handle main menu dynamically
  if (screen == MAIN_MENU) {
    MenuOption mainMenuOptions[6]; // Max possible options
    int count = getCurrentMainMenuOptions(mainMenuOptions);
    
    // Find current option in array
    for (int i = 0; i < count; i++) {
      if (mainMenuOptions[i] == currentOption) {
        // Return previous option (wrap around)
        return mainMenuOptions[(i - 1 + count) % count];
      }
    }
    // If not found, return first option
    return mainMenuOptions[0];
  }

  // Handle mission-specific screens
  if (screen == GUIDED_MODE_CONTROL ||
      screen == AUTO_CONTROL ||
      screen == GO_TO_CONTROL ||
      screen == FOLLOW_CONTROL) {
    if (currentMission) {
      return MissionMenuManager::getInstance().getPreviousMenuOption(currentMission, currentOption);
    }
    return currentOption;
  }
  
  // Handle regular screens
  const MenuOption* options = getMenuOptions(screen);
  int count = getMenuOptionCount(screen);
  
  if (!options || count == 0) {
    return currentOption; // Return current if no options
  }
  
  // Find current option in array
  for (int i = 0; i < count; i++) {
    if (options[i] == currentOption) {
      // Return previous option (wrap around)
      return options[(i - 1 + count) % count];
    }
  }
  
  // If not found, return first option
  return options[0];
}

/**
 * Initialize and enter the menu system
 * Sets menu as active and resets to first option
 */
void enterMenu() {
  menuState.menuActive = true;
  menuState.currentScreen = MAIN_MENU;
  menuState.currentOption = FLIGHT_MODES_MENU;
  menuState.optionCount = getMenuOptionCount(MAIN_MENU);
  screenHistory.clear();
  screenHistory.push_back(MAIN_MENU);
}

/**
 * Check if menu system is currently active
 * @return true if menu is active, false otherwise
 */
bool isInMenu() {
  return menuState.menuActive;
}

/**
 * Get current menu screen
 * @return current MenuScreen
 */
MenuScreen getCurrentScreen() {
  return menuState.currentScreen;
}

/**
 * Navigate to a specific menu screen
 * @param screen The screen to navigate to
 */
void navigateToScreen(MenuScreen screen) {
  screenHistory.push_back(menuState.currentScreen);
  menuState.currentScreen = screen;
  
  // Set appropriate initial option based on screen
  if (screen == GUIDED_MODE_CONTROL ||
      screen == AUTO_CONTROL ||
      screen == GO_TO_CONTROL ||
      screen == FOLLOW_CONTROL) {
    // Mission-specific screens - always create the appropriate mission for the screen
    // Stop any existing mission first
    if (currentMission) {
      currentMission->stop();
      delete currentMission;
      currentMission = nullptr;
    }
    
    // Get flight mode configuration for this screen
    const FlightModeConfig* config = getFlightModeConfigByScreen(screen);
    if (config && config->createMission) {
      currentMission = config->createMission();
    }
    
    if (currentMission) {
      menuState.currentOption = MissionMenuManager::getInstance().getFirstMenuOption(currentMission);
    } else {
      menuState.currentOption = START_MODE; // Default fallback
    }
  } else {
    // Regular screens
    const MenuOption* options = getMenuOptions(screen);
    if (options && getMenuOptionCount(screen) > 0) {
      menuState.currentOption = options[0]; // First option in the array
    } else {
      // Fallback for info screens or invalid screens
      menuState.currentOption = FLIGHT_MODES_MENU;
    }
  }
  
  menuState.optionCount = getMenuOptionCount(screen);
}

/**
 * Go back to previous screen or exit menu
 */
void goBack() {
  if (screenHistory.empty()) {
    // No history, exit menu
    menuState.menuActive = false;
  } else {
    // Go back to previous screen
    MenuScreen previousScreen = screenHistory.back();
    screenHistory.pop_back();
    menuState.currentScreen = previousScreen;
    
    // Set appropriate initial option based on previous screen
    const MenuOption* options = getMenuOptions(previousScreen);
    if (options && getMenuOptionCount(previousScreen) > 0) {
      menuState.currentOption = options[0]; // First option in the array
    } else {
      // Fallback for info screens or invalid screens
      menuState.currentOption = FLIGHT_MODES_MENU;
    }
    
    menuState.optionCount = getMenuOptionCount(previousScreen);
  }
}

/**
 * Cycle to the next menu option
 * Uses the menu option arrays for consistent navigation
 */
void nextMenuOption() {
  menuState.currentOption = getNextMenuOption(menuState.currentScreen, menuState.currentOption);
}

/**
 * Cycle to the previous menu option
 * Uses the menu option arrays for consistent navigation
 */
void previousMenuOption() {
  menuState.currentOption = getPreviousMenuOption(menuState.currentScreen, menuState.currentOption);
}

/**
 * Handle flight modes menu selection
 * @param selectedOption The selected menu option
 */
static void handleFlightModesSelection(MenuOption selectedOption) {
  // Get flight mode configuration for the selected menu option and navigate
  if (const FlightModeConfig* config = getFlightModeConfigByMenuOption(selectedOption)) {
    navigateToScreen(config->controlScreen);
  } else if (selectedOption == BACK) {
    goBack();
  }
}

/**
 * Execute the currently selected menu option
 * Each case handles a specific drone control function
 */
void selectMenuOption() {
  switch (menuState.currentScreen) {
    case MAIN_MENU:
      switch (menuState.currentOption) {
        case FLIGHT_MODES_MENU:
          if (gps_enabled) {
            navigateToScreen(FLIGHT_MODES);
          } else {
            LogProxy::log("GPS must be enabled to access flight modes");
          }
          break;

        case GPS_INFO_SCREEN:
          navigateToScreen(GPS_MENU);
          break;

        case MAVLINK_DETAILS_SCREEN:
          navigateToScreen(MAVLINK_DETAILS);
          break;

        case SETTINGS_MENU:
          navigateToScreen(SETTINGS);
          break;

        case RESTART:
          // Restart the ESP32
          delay(1000);  // Give time for serial message to be sent
          ESP.restart();
          break;

        case EXIT_MENU:
          // Deactivate menu system
          menuState.menuActive = false;
          break;
      }
      break;

    case FLIGHT_MODES:
      handleFlightModesSelection(menuState.currentOption);
      break;

    case FLIGHT_MODE_STATUS:
      switch (menuState.currentOption) {
        case BACK_TO_FLIGHT_MODES:
          goBack();
          break;
      }
      break;

    case SETTINGS:
      switch (menuState.currentOption) {
        case UDP_TOGGLE:
          // Toggle UDP module state from settings menu
          if (udpModule.isEnabled()) {
            udpModule.disable();
            LogProxy::log("UDP disabled");
          } else {
            if (udpModule.enable()) {
              LogProxy::log("UDP enabled");
            } else {
              LogProxy::log("Failed to enable UDP");
            }
          }
          break;
        case GPS_TOGGLE:
          // Toggle GPS module state from settings menu
          gps_enabled = !gps_enabled;
          if (!gps_enabled) {
            // Reset GPS update time when GPS is disabled
            lastGPSUpdate = 0;
            LogProxy::log("GPS disabled");
            // If user is on main menu and currently has Flight Modes selected, move to next valid option
            if (menuState.currentScreen == MAIN_MENU && menuState.currentOption == FLIGHT_MODES_MENU) {
              menuState.currentOption = GPS_INFO_SCREEN;
            }
          } else {
            LogProxy::log("GPS enabled");
          }
          break;
        case GPS_SIMULATION:
          // Toggle GPS simulation mode
          setGPSSimulation(!isGPSSimulationEnabled());
          LogProxy::log(String("GPS simulation ") + (isGPSSimulationEnabled() ? "enabled" : "disabled"));
          break;
        case MAVLINK_DETAILS_TOGGLE:
          // Toggle MAVLink details visibility in main menu
          mavlinkDetailsVisible = !mavlinkDetailsVisible;
          LogProxy::log(String("MAVLink details ") + (mavlinkDetailsVisible ? "shown" : "hidden"));
          break;
        case BACK:
          goBack();
          break;
      }
      break;

    case GPS_MENU:
      switch (menuState.currentOption) {
        case BACK:
          goBack();
          break;
      }
      break;

    case MAVLINK_DETAILS:
      switch (menuState.currentOption) {
        case BACK:
          goBack();
          break;
      }
      break;

    case GUIDED_MODE_CONTROL:
    case AUTO_CONTROL:
    case GO_TO_CONTROL:
    case FOLLOW_CONTROL:
      // Mission-specific screens now delegate to the mission menu manager
      if (currentMission) {
        MissionMenuManager::getInstance().handleMissionMenuAction(currentMission, menuState.currentOption);
        
        // Handle BACK_TO_MODE option
        if (menuState.currentOption == BACK_TO_MODE) {
          goBack();
        }
      }
      break;


  }
}

/**
 * Generate menu display text
 * Creates formatted lines for menu display with current selection indicator
 * @param lines Vector to store the formatted menu lines
 */
void getMenuDisplay(std::vector<String> &lines) {
  switch (menuState.currentScreen) {
    case MAIN_MENU: {
      lines.push_back("== MAIN MENU ==");
      if (gps_enabled) {
        lines.push_back((menuState.currentOption == FLIGHT_MODES_MENU ? "> " : "  ") + String("Flight Modes"));
      }
      lines.push_back((menuState.currentOption == GPS_INFO_SCREEN ? "> " : "  ") + String("GPS Menu"));
      if (mavlinkDetailsVisible) {
        lines.push_back((menuState.currentOption == MAVLINK_DETAILS_SCREEN ? "> " : "  ") + String("MAVLink Details"));
      }
      lines.push_back((menuState.currentOption == SETTINGS_MENU ? "> " : "  ") + String("Settings"));
      lines.push_back((menuState.currentOption == RESTART ? "> " : "  ") + String("Restart"));
      lines.push_back((menuState.currentOption == EXIT_MENU ? "> " : "  ") + String("Exit"));
      break;
    }

    case FLIGHT_MODES: {
      std::vector<int> highlightLines;
      getFlightModesDisplayWithHighlight(lines, highlightLines, getMenuOptionIndex(FLIGHT_MODES, menuState.currentOption));
      break;
    }

    case FLIGHT_MODE_STATUS: {
      lines.push_back("== FLIGHT MODE ==");
      String activeMode = getActiveFlightMode();
      lines.push_back("Mode: " + activeMode);
      lines.push_back("");
      lines.push_back((menuState.currentOption == BACK_TO_FLIGHT_MODES ? "> " : "  ") + String("Back"));
      break;
    }

    case GPS_INFO: {
      lines.push_back("== GPS INFO ==");
      if (gps_enabled) {
        if (gpsHasFix()) {
          lines.push_back("Status: FIX");
          lines.push_back("Sats: " + String(getSatelliteCount()));
          lines.push_back("Updates: " + String(followMeUpdates));
        } else {
          lines.push_back("Status: -");
          lines.push_back("Sats: " + String(getSatelliteCount()));
          if (isGPSStale()) {
            lines.push_back("Signal: LOST");
          } else {
            lines.push_back("Signal: OK");
          }
        }
      }
      
      lines.push_back("");
      lines.push_back("Press to go back");
      break;
    }



    case SETTINGS: {
      lines.push_back("== SETTINGS ==");
      lines.push_back((menuState.currentOption == UDP_TOGGLE ? "> " : "  ") + String("UDP: ") + (udpModule.isEnabled() ? "ON" : "OFF"));
      if (udpModule.isEnabled()) {
        lines.push_back("  Clients: " + String(udpModule.getClientCount()));
      }
      lines.push_back((menuState.currentOption == GPS_TOGGLE ? "> " : "  ") + String("GPS: ") + (gps_enabled ? "ON" : "OFF"));
      lines.push_back((menuState.currentOption == GPS_SIMULATION ? "> " : "  ") + String("GPS Sim: ") + (isGPSSimulationEnabled() ? "ON" : "OFF"));
      lines.push_back((menuState.currentOption == MAVLINK_DETAILS_TOGGLE ? "> " : "  ") + String("MAVLink: ") + (mavlinkDetailsVisible ? "SHOW" : "HIDE"));
      lines.push_back((menuState.currentOption == BACK ? "> " : "  ") + String("Back"));
      break;
    }

    case GPS_MENU: {
      lines.push_back("== GPS INFO ==");
      lines.push_back("Status: " + String(gps_enabled ? "ENABLED" : "DISABLED"));
      if (gps_enabled) {
        lines.push_back("Simulation: " + String(isGPSSimulationEnabled() ? "ON" : "OFF"));
        if (gpsHasFix()) {
          lines.push_back("Fix: YES");
          lines.push_back("Lat: " + String(getLatitude(), 6));
          lines.push_back("Lon: " + String(getLongitude(), 6));
          lines.push_back("Sats: " + String(getSatelliteCount()));
          lines.push_back("Updates: " + String(followMeUpdates));
        } else {
          lines.push_back("Fix: NO");
          lines.push_back("Sats: " + String(getSatelliteCount()));
          if (isGPSStale()) {
            lines.push_back("Signal: LOST");
          } else {
            lines.push_back("Signal: OK");
          }
        }
      }
      lines.push_back("");
      lines.push_back((menuState.currentOption == BACK ? "> " : "  ") + String("Back"));
      break;
    }

    case MAVLINK_DETAILS: {
      lines.push_back("== MAVLINK INFO ==");
      lines.push_back("System ID: " + String(MAVLINK_SYSTEM_ID));
      lines.push_back("Component ID: " + String(MAVLINK_COMPONENT_ID));
      lines.push_back("Target Sys: " + String(MAVLINK_TARGET_SYSTEM_ID));
      lines.push_back("Target Comp: " + String(MAVLINK_TARGET_COMPONENT_ID));
      lines.push_back("Armed: " + String(armed ? "YES" : "NO"));
      lines.push_back("");
      lines.push_back((menuState.currentOption == BACK ? "> " : "  ") + String("Back"));
      break;
    }

    case GUIDED_MODE_CONTROL:
    case AUTO_CONTROL:
    case GO_TO_CONTROL:
    case FOLLOW_CONTROL:
      // Mission-specific screens now delegate to the mission menu manager
      if (currentMission) {
        MissionMenuManager::getInstance().getMissionMenuDisplay(currentMission, lines, menuState.currentOption);
      } else {
        lines.push_back("No active mission");
      }
      break;


  }
}

/**
 * Generate menu display text with highlight information
 * Creates formatted lines for menu display with current selection indicator
 * and identifies lines that should be highlighted (active flight modes)
 * @param lines Vector to store the formatted menu lines
 * @param highlightLines Vector to store line indices to highlight
 */
void getMenuDisplayWithHighlight(std::vector<String> &lines, std::vector<int> &highlightLines) {
  // First get the normal menu display
  getMenuDisplay(lines);
  
  // Add highlight information for flight modes menu
  if (menuState.currentScreen == FLIGHT_MODES) {
    // Clear the lines and rebuild with highlight information
    lines.clear();
    getFlightModesDisplayWithHighlight(lines, highlightLines, getMenuOptionIndex(FLIGHT_MODES, menuState.currentOption));
  }
} 