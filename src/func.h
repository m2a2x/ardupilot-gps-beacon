#ifndef FUNC_H
#define FUNC_H

#include <Arduino.h>
#include <WiFiUdp.h>
#include <mavlink/v2.0/common/mavlink.h>

// Constants
extern const uint16_t groundstation_port;
extern int16_t system_mode;
extern int16_t control_mode;
extern const int16_t system_id;
extern const int16_t component_id;
extern unsigned long previousTimeoutMillis;

// Function declarations
bool sendUDP(const uint8_t* data, size_t len, IPAddress ip);
void send_mavlink(mavlink_message_t *mavlink_message, const char* groundstation_host);
void send_heartbeat(IPAddress targetIP);
void send_systemstatus(const char* groundstation_host);
void send_radiostatus(const char* groundstation_host);
void send_followme(float lat_deg, float lon_deg, float alt_m, IPAddress targetIP);
void send_position(IPAddress targetIP);
void handle_message_command_long(mavlink_message_t *msg, const char* groundstation_host);
void handle_mission_manual_control(mavlink_message_t *msg);
void parse_mavlink(uint8_t parsing_byte, const char* groundstation_host);

#endif // FUNC_H