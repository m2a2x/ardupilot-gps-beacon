#pragma once
#include <Arduino.h>
#include <vector>
#include <mavlink/v2.0/common/mavlink.h>
#include "mission/mission.h"

/**
 * Structure to store MAVLink message information
 */
struct MavlinkMessageInfo {
  uint32_t timestamp;     ///< Message timestamp
  uint8_t msgid;          ///< MAVLink message ID
  uint8_t system_id;      ///< Source system ID
  uint8_t component_id;   ///< Source component ID
  String description;     ///< Human-readable description
  bool is_valid;          ///< Whether the message is valid
  std::vector<String> fields; ///< Parsed field names and values
};

/**
 * Structure to store parsed MAVLink field information
 */
struct MavlinkField {
  String name;    ///< Field name
  String value;   ///< Field value as string
  String type;    ///< Field data type
};

/**
 * Calculate velocity between two GPS points
 * @param lat1 First latitude
 * @param lon1 First longitude
 * @param alt1 First altitude
 * @param lat2 Second latitude
 * @param lon2 Second longitude
 * @param alt2 Second altitude
 * @param dt Time difference in seconds
 * @param vx Output X velocity (m/s)
 * @param vy Output Y velocity (m/s)
 * @param vz Output Z velocity (m/s)
 */
void calculateVelocity(double lat1, double lon1, float alt1,
                      double lat2, double lon2, float alt2,
                      float dt, float &vx, float &vy, float &vz);

/**
 * Add a new MAVLink message to the message history
 * @param msgid MAVLink message ID
 * @param system_id Source system ID
 * @param component_id Source component ID
 * @param description Human-readable description of the message
 */
void addMavlinkMessage(uint8_t msgid, uint8_t system_id, uint8_t component_id, const String& description);

/**
 * Add a new MAVLink message to the message history with parsed fields
 * @param msg MAVLink message to add
 */
void addMavlinkMessage(const mavlink_message_t& msg);

/**
 * Get human-readable description for a MAVLink message ID
 * @param msgid MAVLink message ID
 * @return String description of the message type
 */
String getMavlinkMessageDescription(uint8_t msgid);

/**
 * Get short message name for compact display
 * @param msgid MAVLink message ID
 * @return Short name for the message type
 */
String getShortMessageName(uint8_t msgid);

/**
 * Get MAVLink message display
 * @param lines Vector to store MAVLink message display lines
 */
void getMavlinkMessagesDisplay(std::vector<String>& lines);

/**
 * Clear all MAVLink messages from history
 * This function can be called to clear previous messages when rendering
 */
void clearMavlinkMessages();

/**
 * Update current mission
 * This function should be called regularly in the main loop
 * to update the currently active mission
 */
void updateCurrentMission();

/**
 * Get the name of the currently active flight mode
 * @return String representing the active flight mode, or empty string if none
 */
String getActiveFlightMode();

/**
 * Get mission status display
 * @param lines Vector to store mission display lines
 */
void getMissionStatusDisplay(std::vector<String>& lines);

/**
 * Parse MAVLink message and extract all fields with their values
 * @param msg MAVLink message to parse
 * @param fields Vector to store parsed field information
 */
void parseMavlinkMessage(const mavlink_message_t& msg, std::vector<MavlinkField>& fields);

/**
 * Get the latest battery info from MAVLink SYS_STATUS message
 * @param voltage_battery (out) Battery voltage in volts
 * @param current_battery (out) Battery current in amps
 * @param battery_remaining (out) Battery remaining percentage
 * @return true if found, false otherwise
 */
bool getLatestBatteryInfo(float &voltage_battery, float &current_battery, int &battery_remaining);

/**
 * Get the latest flight mode from MAVLink HEARTBEAT message
 * @param mode (out) Human-readable flight mode string
 * @return true if found, false otherwise
 */
bool getLatestFlightMode(String &mode);

// External declarations
extern Mission* currentMission;

// Mission management
void getActiveFlightModeDisplay(std::vector<String> &lines); 