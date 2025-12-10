#pragma once
#include <Arduino.h>
#include <vector>
#include "menu_types.h"
#include "mission/mission.h"
#include "utils.h"





/**
 * Menu navigation structure
 */
struct MenuState {
  MenuScreen currentScreen;
  MenuOption currentOption;
  bool menuActive;
  int optionCount;  // Number of options in current screen
};

// Menu option arrays for each screen
extern const MenuOption MAIN_MENU_OPTIONS[];
extern const MenuOption FLIGHT_MODES_OPTIONS[];
extern const MenuOption FLIGHT_MODE_STATUS_OPTIONS[];
extern const MenuOption SETTINGS_OPTIONS[];
extern const MenuOption GPS_MENU_OPTIONS[];
extern const MenuOption MAVLINK_DETAILS_OPTIONS[];
extern const MenuOption FLIGHT_MODE_CONTROL_OPTIONS[];

/**
 * Get menu options for a specific screen
 * @param screen The screen to get options for
 * @return Array of menu options for the screen
 */
const MenuOption* getMenuOptions(MenuScreen screen);

/**
 * Get the number of options for a specific screen
 * @param screen The screen to get option count for
 * @return Number of options in the screen
 */
int getMenuOptionCount(MenuScreen screen);

/**
 * Get the next option in sequence for a screen
 * @param screen The current screen
 * @param currentOption The current option
 * @return The next option in sequence
 */
MenuOption getNextMenuOption(MenuScreen screen, MenuOption currentOption);

/**
 * Get the previous option in sequence for a screen
 * @param screen The current screen
 * @param currentOption The current option
 * @return The previous option in sequence
 */
MenuOption getPreviousMenuOption(MenuScreen screen, MenuOption currentOption);

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