#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>

// Network configuration
#define LOCAL_IP 192,168,4,1
#define GROUNDSTATION_IP 192,168,4,1  // IP of QGroundControl machine
#define GROUNDSTATION_PORT 14550
#define MAVLINK_BAUD 57600


// Common MAVLink system and component IDs
extern const uint8_t MAVLINK_SYSTEM_ID;
extern const uint8_t MAVLINK_COMPONENT_ID;

// Common MAVLink target system and component IDs
extern const uint8_t MAVLINK_TARGET_SYSTEM_ID;
extern const uint8_t MAVLINK_TARGET_COMPONENT_ID;


// Wi-Fi AP configuration
extern const char *ap_ssid;
extern const char *ap_pass;

// Button configuration
const int BUTTON_PIN = 0;  // Using ESP32's built-in BOOT button (GPIO 0)
const int LONG_PRESS_MS = 800;

// Note: BOOT button (GPIO 0) is pulled up internally and goes LOW when pressed
// The button is also used for entering download mode when held during boot
// For normal operation, it should work fine for menu navigation
// If BOOT button doesn't work, try using GPIO 2, 4, or 5 instead

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
extern bool wifi_enabled;
extern bool gps_enabled;

extern WiFiUDP udp;

#define FOLLOW_ALT 2.0f

// === Hardware Serial Configuration ===
extern HardwareSerial mavSerial;
extern bool armed;

