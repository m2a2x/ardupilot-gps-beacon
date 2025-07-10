#pragma once
#include <Arduino.h>
#include "conf.h"  // For BUTTON_PIN and LONG_PRESS_MS
#include "menu/menu.h"  // For menu functions

// Forward declarations
void enterMenu();
bool isInMenu();
void selectMenuOption();
void nextMenuOption();

/**
 * Initialize button hardware
 */
void setupButton();

/**
 * Handle button input and trigger appropriate actions
 * - Short press: Enter menu or next option
 * - Long press: Select current option
 */
void handleButton();

