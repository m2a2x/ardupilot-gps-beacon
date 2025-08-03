#include "button.h"
#include <Arduino.h>
#include "conf.h"  // For BUTTON_PIN, RTL_BUTTON_PIN and LONG_PRESS_MS
#include "menu/menu.h"  // For menu functions
#include "log_proxy.h"  // For logging
#include "utils.h"  // For currentMission
#include "mavlink_cmds.h"  // For RTL mode command

// Button state variables
static unsigned long pressStartTime = 0;
static bool lastButtonState = HIGH;
static bool longPressTriggered = false;
static const unsigned long DEBOUNCE_DELAY = 50;  // 50ms debounce time
static const unsigned long CLICK_TIMEOUT_MS = 1000; // 1 second timeout for click actions
static unsigned long lastDebounceTime = 0;
static bool buttonState = HIGH;
static bool lastStableState = HIGH;

// Double-click detection
static unsigned long lastClickTime = 0;
static bool doubleClickDetected = false;
static const unsigned long DOUBLE_CLICK_TIMEOUT = 300; // 300ms for double-click



void setupButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RTL_BUTTON_PIN, INPUT_PULLUP);
  
  // Test the button state on startup
  bool initialState = digitalRead(BUTTON_PIN);
  // Small delay to let button settle, then test again
  delay(10);
  bool testState = digitalRead(BUTTON_PIN);
  
  // Initialize static variables to match current state
  buttonState = testState;
  lastStableState = testState;
  lastButtonState = testState;
}

void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  if (reading != lastButtonState) {
    lastDebounceTime = now;
  }

  if ((now - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == LOW) {  // Button pressed
        pressStartTime = now;
        longPressTriggered = false;
        doubleClickDetected = false;
      } else {  // Button released
        unsigned long pressDuration = now - pressStartTime;
        
        if (pressDuration >= LONG_PRESS_MS && !longPressTriggered) {
          // Long press action
          if (isInMenu()) {
            selectMenuOption();
          }
        } else if (!longPressTriggered) {
          // Short press action - check for double-click
          if ((now - lastClickTime) < DOUBLE_CLICK_TIMEOUT) {
            // Double-click detected
            doubleClickDetected = true;
            if (isInMenu()) {
              goBack(); // Go back to previous screen
            }
          } else {
            // Single click
            if (isInMenu()) {
              nextMenuOption();
            } else {
              enterMenu();
            }
          }
          lastClickTime = now;
        }
      }
    }
  }

  // Handle long press while button is held
  if (buttonState == LOW) {
    unsigned long pressDuration = now - pressStartTime;
    if (pressDuration >= LONG_PRESS_MS && !longPressTriggered) {
      longPressTriggered = true;
      if (isInMenu()) {
        selectMenuOption();
      }
    }
  }

  lastButtonState = reading;
}

void handleRTLButton() {
  // Simple check - if button is pressed (LOW), send RTL command
  if (digitalRead(RTL_BUTTON_PIN) == LOW) {
    // Stop any active mission first
    if (currentMission != nullptr) {
      currentMission->stop();
    }
    // Send RTL command
    send_set_mode_command("RTL");
  }
} 