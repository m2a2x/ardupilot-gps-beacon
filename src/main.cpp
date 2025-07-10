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
#include "clients.h" // Client management
#include "menu/menu.h"    // Menu system
#include "button.h"  // Button handling
#include "gps.h"     // GPS functionality
#include "tasks.h"   // FreeRTOS tasks
#include "utils.h"   // Utility functions
#include "proxy.h"   // MAVLink proxy
#include "udp_module.h"   // UDP module

// === Display and Status Variables ===
StatusDisplay oled;
unsigned long rxBytes = 0, txBytes = 0;

// === UDP Configuration ===
// UDP module is handled by udpModule instance

/**
 * Initialize all hardware and network components
 */
void setup() {
    // Initialize serial communications
    Serial.begin(MAVLINK_BAUD);
    mavSerial.begin(MAVLINK_BAUD, SERIAL_8N1, RXD2, TXD2);
    
    // Initialize GPS
    if (!setupGPS()) {
        Serial.println("Error: GPS initialization failed!");
    }
    
    // Initialize button and display
    setupButton();
    if (!oled.begin()) {
        Serial.println("Error: Display initialization failed!");
    }

    // Initialize UDP module (disabled by default)
    if (!udpModule.begin()) {
        Serial.println("Error: UDP module initialization failed!");
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