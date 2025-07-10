#include "conf.h"
#include <WiFiUdp.h>

// Wi-Fi AP configuration
const char *ap_ssid = "ESP32-MAVLink";
const char *ap_pass = "mavlink123";

// Common MAVLink system and component IDs
const uint8_t MAVLINK_SYSTEM_ID = 15;
const uint8_t MAVLINK_COMPONENT_ID = 200;

// Common MAVLink target system and component IDs
const uint8_t MAVLINK_TARGET_SYSTEM_ID = 1;
const uint8_t MAVLINK_TARGET_COMPONENT_ID = 0;

// Global UDP instance
WiFiUDP udp;
bool armed = false;

// State tracking
bool wifi_enabled = true;
bool gps_enabled = true; 

// Hardware Serial instance
HardwareSerial mavSerial(1); 