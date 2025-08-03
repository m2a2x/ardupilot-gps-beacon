#pragma once
#include <Arduino.h>

// Debug configuration
#define DEBUG false
// Network configuration
#define GROUNDSTATION_IP 192,168,4,1  // IP of QGroundControl machine
#define MAVLINK_BAUD 57600
#define GPS_BAUD 9600

// Common MAVLink system and component IDs
extern const uint8_t MAVLINK_SYSTEM_ID;
extern const uint8_t MAVLINK_COMPONENT_ID;

// Common MAVLink target system and component IDs
extern const uint8_t MAVLINK_TARGET_SYSTEM_ID;
extern const uint8_t MAVLINK_TARGET_COMPONENT_ID;

// Button configuration
const int BUTTON_PIN = 25;  // Using GPIO 25 for button input
const int RTL_BUTTON_PIN = 27;  // Using GPIO 27 for RTL button input
const int LONG_PRESS_MS = 800;

// Note: GPIO 25 is used for button input
// Note: GPIO 27 is used for RTL button input
// The buttons should be connected with a pull-up resistor (or use INPUT_PULLUP mode)
// Buttons go LOW when pressed

// Serial configuration
const int RXD2 = 17;
const int TXD2 = 16;

// Display configuration
const int SDA_PIN = 22;
const int SCL_PIN = 23;

// GPS configuration
const int GPS_RX_PIN = 18;
const int GPS_TX_PIN = 19;

// State tracking
extern bool gps_enabled;

#define FOLLOW_ALT 2.0f

// === Hardware Serial Configuration ===
extern HardwareSerial mavSerial;
extern bool armed;

