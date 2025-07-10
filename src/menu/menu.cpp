#include "menu.h"
#include "conf.h"  // For configuration constants
#include "gps.h"   // For GPS functions
#include "mavlink_cmds.h"  // For MAVLink commands
#include "battery.h"  // For battery functions
#include "clients.h"  // For GCSClient
#include "mission/mission.h"  // For mission management
#include "mission/mission_loiter.h"
#include "mission/mission_guided.h"
#include "mission/mission_followme.h"
#include "mission/mission_goto.h"
#include "mission/mission_arm.h"
#include "utils.h"  // For utility functions
#include "udp_module.h"  // For UDP module

// External declarations
extern unsigned long rxBytes, txBytes;  // From main.cpp
extern unsigned long lastGPSUpdate;     // From gps.cpp
extern int8_t radio_rssi;               // From tasks.cpp
extern std::vector<GCSClient> clients;  // From clients.h

// Menu state variables
static MenuState menuState = {
  .currentScreen = MAIN_MENU,
  .currentOption = FLIGHT_MODES_MENU,
  .menuActive = false,
  .optionCount = 0
};

// Navigation history for back button functionality
static std::vector<MenuScreen> screenHistory;

// Follow me status
uint32_t followMeUpdates = 0;
bool followMeEnabled = false;
static bool followMeActive = false;  // Tracks if FollowMe mode is active

// Previous GPS position for velocity calculation
static double lastLat = 0.0;
static double lastLon = 0.0;
static float lastAlt = 0.0;
static unsigned long lastPosTime = 0;

/**
 * Get the number of options for a given screen
 * @param screen The screen to get option count for
 * @return Number of options in the screen
 */
int getOptionCount(MenuScreen screen) {
  switch (screen) {
    case MAIN_MENU:
      return 7; // FLIGHT_MODES_MENU, GPS_INFO_SCREEN, MAVLINK_MESSAGES_SCREEN, SETTINGS_MENU, RESTART, STOP_FLIGHT, EXIT_MENU
    case FLIGHT_MODES:
      return 7; // LOITER_MODE, GUIDED_MODE, FOLLOW_ME, GO_TO, ARM, STOP_MISSION, BACK
    case SETTINGS:
      return 2; // TOGGLE_WIFI_SETTINGS, BACK
    case GPS_MENU:
      return 2; // TOGGLE_GPS, BACK
    default:
      return 1; // Info screens have no options, just display
  }
}

/**
 * Initialize and enter the menu system
 * Sets menu as active and resets to first option
 */
void enterMenu() {
  menuState.menuActive = true;
  menuState.currentScreen = MAIN_MENU;
  menuState.currentOption = FLIGHT_MODES_MENU;
  menuState.optionCount = getOptionCount(MAIN_MENU);
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
  menuState.currentOption = FLIGHT_MODES_MENU; // Reset to first option
  menuState.optionCount = getOptionCount(screen);
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
    menuState.currentOption = FLIGHT_MODES_MENU; // Reset to first option
    menuState.optionCount = getOptionCount(previousScreen);
  }
}

/**
 * Cycle to the next menu option
 * Uses modulo arithmetic to wrap around to first option
 */
void nextMenuOption() {
  if (menuState.currentScreen == MAIN_MENU) {
    // Main menu has a specific sequence due to unused MAVLINK_DETAILS_SCREEN
    int current = static_cast<int>(menuState.currentOption);
    
    // Define the correct sequence for main menu options
    switch (current) {
      case 0: // FLIGHT_MODES_MENU
        current = 1; // GPS_INFO_SCREEN
        break;
      case 1: // GPS_INFO_SCREEN
        current = 2; // MAVLINK_MESSAGES_SCREEN
        break;
      case 2: // MAVLINK_MESSAGES_SCREEN
        current = 4; // SETTINGS_MENU (skip 3)
        break;
      case 4: // SETTINGS_MENU
        current = 5; // RESTART
        break;
      case 5: // RESTART
        current = 6; // STOP_FLIGHT
        break;
      case 6: // STOP_FLIGHT
        current = 7; // EXIT_MENU
        break;
      case 7: // EXIT_MENU
        current = 0; // Back to FLIGHT_MODES_MENU
        break;
      default:
        // If we somehow get an invalid value, reset to first option
        current = 0;
        break;
    }
    
    menuState.currentOption = static_cast<MenuOption>(current);
  } else if (menuState.currentScreen == FLIGHT_MODES) {
    // Flight modes sub-menu has special handling
    int current = static_cast<int>(menuState.currentOption);
    current = (current + 1) % menuState.optionCount;
    menuState.currentOption = static_cast<MenuOption>(current);
  } else if (menuState.currentScreen == SETTINGS) {
    // Settings sub-menu has special handling
    int current = static_cast<int>(menuState.currentOption);
    current = (current + 1) % menuState.optionCount;
    menuState.currentOption = static_cast<MenuOption>(current);
  } else if (menuState.currentScreen == GPS_MENU) {
    // GPS menu has special handling
    int current = static_cast<int>(menuState.currentOption);
    current = (current + 1) % menuState.optionCount;
    menuState.currentOption = static_cast<MenuOption>(current);
  }
  // Info screens don't have selectable options
}

/**
 * Cycle to the previous menu option
 * Uses modulo arithmetic to wrap around to last option
 */
void previousMenuOption() {
  if (menuState.currentScreen == MAIN_MENU) {
    // Main menu has a specific sequence due to unused MAVLINK_DETAILS_SCREEN
    int current = static_cast<int>(menuState.currentOption);
    
    // Define the correct reverse sequence for main menu options
    switch (current) {
      case 0: // FLIGHT_MODES_MENU
        current = 7; // EXIT_MENU
        break;
      case 1: // GPS_INFO_SCREEN
        current = 0; // FLIGHT_MODES_MENU
        break;
      case 2: // MAVLINK_MESSAGES_SCREEN
        current = 1; // GPS_INFO_SCREEN
        break;
      case 4: // SETTINGS_MENU
        current = 2; // MAVLINK_MESSAGES_SCREEN (skip 3)
        break;
      case 5: // RESTART
        current = 4; // SETTINGS_MENU
        break;
      case 6: // STOP_FLIGHT
        current = 5; // RESTART
        break;
      case 7: // EXIT_MENU
        current = 6; // STOP_FLIGHT
        break;
      default:
        // If we somehow get an invalid value, reset to first option
        current = 0;
        break;
    }
    
    menuState.currentOption = static_cast<MenuOption>(current);
  } else if (menuState.currentScreen == FLIGHT_MODES) {
    // Flight modes sub-menu has special handling
    int current = static_cast<int>(menuState.currentOption);
    current = (current - 1 + menuState.optionCount) % menuState.optionCount;
    menuState.currentOption = static_cast<MenuOption>(current);
  } else if (menuState.currentScreen == SETTINGS) {
    // Settings sub-menu has special handling
    int current = static_cast<int>(menuState.currentOption);
    current = (current - 1 + menuState.optionCount) % menuState.optionCount;
    menuState.currentOption = static_cast<MenuOption>(current);
  } else if (menuState.currentScreen == GPS_MENU) {
    // GPS menu has special handling
    int current = static_cast<int>(menuState.currentOption);
    current = (current - 1 + menuState.optionCount) % menuState.optionCount;
    menuState.currentOption = static_cast<MenuOption>(current);
  }
  // Info screens don't have selectable options
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
          navigateToScreen(FLIGHT_MODES);
          break;

        case GPS_INFO_SCREEN:
          navigateToScreen(GPS_MENU);
          break;

        case MAVLINK_MESSAGES_SCREEN:
          navigateToScreen(MAVLINK_MESSAGES);
          break;

        case MAVLINK_DETAILS_SCREEN:
          // This should not be reachable with the current navigation fix
          // but handle it gracefully by going to MAVLINK_MESSAGES
          navigateToScreen(MAVLINK_MESSAGES);
          break;

        case SETTINGS_MENU:
          navigateToScreen(SETTINGS);
          break;

        case RESTART:
          // Restart the ESP32
          Serial.println("Restarting...");
          delay(1000);  // Give time for serial message to be sent
          ESP.restart();
          break;

        case STOP_FLIGHT:
          // Stop flight and hold position
          stopMissionAndLoiter();
          Serial.println("Stop flight executed - drone holding position");
          break;

        case EXIT_MENU:
          // Deactivate menu system
          menuState.menuActive = false;
          Serial.println("Exited menu");
          break;
      }
      break;

    case FLIGHT_MODES:
      switch (menuState.currentOption) {
        case 0: // LOITER_MODE
          if (currentMission) { currentMission->stop(); delete currentMission; currentMission = nullptr; }
          currentMission = new LoiterMission();
          currentMission->start();
          goBack();
          break;
        case 1: // GUIDED_MODE
          if (currentMission) { currentMission->stop(); delete currentMission; currentMission = nullptr; }
          currentMission = new GuidedMission();
          currentMission->start();
          goBack();
          break;
        case 2: // FOLLOW_ME
          if (currentMission) { currentMission->stop(); delete currentMission; currentMission = nullptr; }
          currentMission = new FollowMeMission();
          currentMission->start();
          goBack();
          break;
        case 3: // GO_TO
          if (currentMission) { currentMission->stop(); delete currentMission; currentMission = nullptr; }
          currentMission = new GoToMission();
          currentMission->start();
          goBack();
          break;
        case 4: // ARM
          if (currentMission) { currentMission->stop(); delete currentMission; currentMission = nullptr; }
          currentMission = new ArmMission();
          currentMission->start();
          goBack();
          break;
        case 5: // STOP_MISSION
          stopMissionAndLoiter();
          goBack();
          break;
        case 6: // BACK
          goBack();
          break;
      }
      break;

    case SETTINGS:
      switch (menuState.currentOption) {
        case 0: // TOGGLE_WIFI_SETTINGS
          // Toggle UDP module state from settings menu
          if (udpModule.isEnabled()) {
            udpModule.disable();
            // Reset communication counters when UDP is disabled
            rxBytes = 0;
            txBytes = 0;
            Serial.println("UDP disabled (from settings)");
          } else {
            if (udpModule.enable()) {
              Serial.println("UDP enabled (from settings)");
            } else {
              Serial.println("Failed to enable UDP (from settings)");
            }
          }
          break;
        case 1: // BACK
          goBack();
          break;
      }
      break;

    case GPS_MENU:
      switch (menuState.currentOption) {
        case 0: // TOGGLE_GPS
          // Toggle GPS module state from GPS menu
          gps_enabled = !gps_enabled;
          if (!gps_enabled) {
            // Reset GPS update time when GPS is disabled
            lastGPSUpdate = 0;
          }
          Serial.println(gps_enabled ? "GPS enabled (from GPS menu)" : "GPS disabled (from GPS menu)");
          break;
        case 1: // BACK
          goBack();
          break;
      }
      break;

    default:
      // Info screens - just go back
      goBack();
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
      lines.push_back((menuState.currentOption == FLIGHT_MODES_MENU ? "> " : "  ") + String("Flight Modes"));
      lines.push_back((menuState.currentOption == GPS_INFO_SCREEN ? "> " : "  ") + String("GPS Menu"));
      lines.push_back((menuState.currentOption == MAVLINK_MESSAGES_SCREEN ? "> " : "  ") + String("Statuses"));
      lines.push_back((menuState.currentOption == SETTINGS_MENU ? "> " : "  ") + String("Settings"));
      lines.push_back((menuState.currentOption == RESTART ? "> " : "  ") + String("Restart"));
      lines.push_back((menuState.currentOption == STOP_FLIGHT ? "> " : "  ") + String("Stop Flight"));
      lines.push_back((menuState.currentOption == EXIT_MENU ? "> " : "  ") + String("Exit"));
      break;
    }

    case FLIGHT_MODES: {
      lines.push_back("== FLIGHT MODES ==");
      lines.push_back((menuState.currentOption == 0 ? "> " : "  ") + String("Loiter Mode"));
      lines.push_back((menuState.currentOption == 1 ? "> " : "  ") + String("Guided Mode"));
      lines.push_back((menuState.currentOption == 2 ? "> " : "  ") + String("Follow Me"));
      lines.push_back((menuState.currentOption == 3 ? "> " : "  ") + String("Go To"));
      lines.push_back((menuState.currentOption == 4 ? "> " : "  ") + String("Arm"));
      lines.push_back((menuState.currentOption == 5 ? "> " : "  ") + String("Stop Mission"));
      lines.push_back((menuState.currentOption == 6 ? "> " : "  ") + String("Back"));
      break;
    }

    case GPS_INFO: {
      lines.push_back("== GPS INFO ==");
      if (gps_enabled) {
        if (gpsHasFix()) {
          lines.push_back("Status: FIX");
          lines.push_back("Lat: " + String(getLatitude(), 6));
          lines.push_back("Lon: " + String(getLongitude(), 6));
          lines.push_back("Alt: " + String(getAltitude(), 1) + "m");
          lines.push_back("Sats: " + String(getSatelliteCount()));
          lines.push_back("Follow Me: " + String(followMeUpdates));
        } else {
          lines.push_back("Status: NO FIX");
          lines.push_back("Sats: " + String(getSatelliteCount()));
          if (isGPSStale()) {
            lines.push_back("Signal: LOST");
          } else {
            lines.push_back("Signal: OK");
          }
        }
      } else {
        lines.push_back("GPS: DISABLED");
      }
      lines.push_back("");
      lines.push_back("Press to go back");
      break;
    }

    case MAVLINK_MESSAGES:
      getMavlinkMessagesDisplay(lines);
      break;

    case SETTINGS: {
      lines.push_back("== SETTINGS ==");
      lines.push_back((menuState.currentOption == 0 ? "> " : "  ") + String("UDP ") + (udpModule.isEnabled() ? "OFF" : "ON"));
      lines.push_back((menuState.currentOption == 1 ? "> " : "  ") + String("Back"));
      break;
    }

    case GPS_MENU: {
      lines.push_back("== GPS MENU ==");
      if (gps_enabled) {
        if (gpsHasFix()) {
          lines.push_back("Status: FIX");
          lines.push_back("Lat: " + String(getLatitude(), 6));
          lines.push_back("Lon: " + String(getLongitude(), 6));
          lines.push_back("Alt: " + String(getAltitude(), 1) + "m");
          lines.push_back("Sats: " + String(getSatelliteCount()));
          lines.push_back("Follow Me: " + String(followMeUpdates));
        } else {
          lines.push_back("Status: NO FIX");
          lines.push_back("Sats: " + String(getSatelliteCount()));
          if (isGPSStale()) {
            lines.push_back("Signal: LOST");
          } else {
            lines.push_back("Signal: OK");
          }
        }
      } else {
        lines.push_back("GPS: DISABLED");
      }
      lines.push_back("");
      lines.push_back((menuState.currentOption == 0 ? "> " : "  ") + String("GPS ") + (gps_enabled ? "OFF" : "ON"));
      lines.push_back((menuState.currentOption == 1 ? "> " : "  ") + String("Back"));
      break;
    }
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
    String activeMode = getActiveFlightMode();
    
    // Check each flight mode line (lines 1, 2, 3, 4 after the header)
    if (activeMode == "Loiter Mode") {
      highlightLines.push_back(1); // Loiter Mode line
    } else if (activeMode == "Guided Mode") {
      highlightLines.push_back(2); // Guided Mode line
    } else if (activeMode == "Follow Me") {
      highlightLines.push_back(3); // Follow Me line
    } else if (activeMode == "GoTo") {
      highlightLines.push_back(4); // GoTo line
    } else if (activeMode == "Arm") {
      highlightLines.push_back(5); // Arm line
    }
  }
} 