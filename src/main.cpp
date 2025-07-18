/**
 * MAVLink Bridge for ESP32
 * 
 * This program creates a WiFi access point and bridges MAVLink messages
 * between UDP (for ground station) and UART (for flight controller).
 * It also provides GPS data and status display functionality.
 */

#include <Arduino.h>
#include <TinyGPS++.h>
#include <mavlink/v2.0/common/mavlink.h>
#include "conf.h"    // Configuration constants
#include "display.h" // Display abstraction
// Client management is now handled by UDP module
#include "menu/menu.h"    // Menu system
#include "button.h"  // Button handling
#include "gps.h"     // GPS functionality
#include "tasks.h"   // FreeRTOS tasks
#include "utils.h"   // Utility functions
#include "radio.h"   // MAVLink radio
#include "udp_module.h"   // UDP module
#include "log_proxy.h"   // Log proxy

// === Display and Status Variables ===
StatusDisplay oled;
/**
 * Initialize all hardware and network components
 */
void setup() {
    // Initialize serial communications
    Serial.begin(MAVLINK_BAUD);
    mavSerial.begin(MAVLINK_BAUD, SERIAL_8N1, RXD2, TXD2);
    
    // Initialize GPS
    if (!setupGPS()) {
        LogProxy::log("Error: GPS initialization failed!");
    }
    
    // Initialize button and display
    setupButton();
    if (!oled.begin()) {
        LogProxy::log("Error: Display initialization failed!");
    }

    // Initialize UDP module (disabled by default)
    if (!udpModule.begin()) {
        LogProxy::log("Error: UDP module initialization failed!");
    }

    // Initialize FreeRTOS tasks
    initTasks();
}

/**
 * Main program loop
 * In FreeRTOS, this is not used as tasks handle all functionality
 */
void loop() {
    // This function is not used in FreeRTOS implementation
    vTaskDelete(NULL);
}