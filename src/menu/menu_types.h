#pragma once

/**
 * Menu screen types for multi-level navigation
 */
enum MenuScreen {
  MAIN_MENU,      ///< Main menu with primary options
  GPS_INFO,       ///< GPS information screen
  MAVLINK_DETAILS, ///< MAVLink messages screen (detailed view with fields)
  FLIGHT_MODES,   ///< Flight mode selection sub-menu
  FLIGHT_MODE_STATUS, ///< Individual flight mode status screen
  SETTINGS,       ///< Settings sub-menu
  GPS_MENU,       ///< GPS control sub-menu
  GUIDED_MODE_CONTROL, ///< Guided mode control screen (start/stop/back)
  AUTO_CONTROL, ///< Auto mode control screen (start/stop/back)
  GO_TO_CONTROL,       ///< Go to mode control screen (start/stop/back)
  FOLLOW_CONTROL,      ///< Follow mode control screen (start/stop/back)
};

/**
 * Available menu options for drone control
 * Each option represents a different control function
 */
enum MenuOption {
  FLIGHT_MODES_MENU, ///< Enter flight modes sub-menu
  GPS_INFO_SCREEN,   ///< Enter GPS info screen
  MAVLINK_DETAILS_SCREEN, ///< Enter detailed MAVLink messages screen
  SETTINGS_MENU,     ///< Enter settings sub-menu
  RESTART,        ///< Restart the ESP32
  EXIT_MENU,      ///< Exit menu system and return to normal operation
  
  // Flight modes sub-menu options
  GUIDED_MODE,    ///< Enter guided mode control
  AUTO, ///< Enter auto mode control
  GO_TO,          ///< Enter go to mode control
  FOLLOW,         ///< Enter follow mode control
  BACK,           ///< Go back to previous menu
  
  // Flight mode status screen options
  BACK_TO_FLIGHT_MODES, ///< Go back to flight modes menu
  
  // Individual flight mode control options
  START_MODE,     ///< Start the current flight mode
  STOP_MODE,      ///< Stop the current flight mode
  RTL_MODE,       ///< Set drone to RTL mode and complete mission
  ALT_3M,         ///< Set altitude to 3 meters
  ALT_6M,         ///< Set altitude to 6 meters
  ALT_9M,         ///< Set altitude to 8 meters
  ROI_CONTROL,    ///< Toggle ROI control (drone always points to beacon)
  BACK_TO_MODE,   ///< Go back to flight modes menu
  
  // Settings menu options
  UDP_TOGGLE,     ///< Toggle UDP/WiFi on/off
  GPS_TOGGLE,     ///< Toggle GPS on/off
  GPS_SIMULATION, ///< Toggle GPS simulation mode
  MAVLINK_DETAILS_TOGGLE ///< Toggle MAVLink details visibility in main menu
}; 