#pragma once
#include <Arduino.h>
#include <mavlink/v2.0/common/mavlink.h>
#include "flight_modes.h"

// Common MAVLink system and component IDs
extern const uint8_t MAVLINK_SYSTEM_ID;
extern const uint8_t MAVLINK_COMPONENT_ID;

// Common MAVLink target system and component IDs
extern const uint8_t MAVLINK_TARGET_SYSTEM_ID;
extern const uint8_t MAVLINK_TARGET_COMPONENT_ID;



/**
 * Send a command to set the flight mode using mode name
 * This function sends a MAVLink SET_MODE message to the autopilot
 * using a human-readable mode name
 * 
 * @param mode_name Flight mode name (e.g., "GUIDED", "FOLLOW", "AUTO", "LOITER")
 */
void send_set_mode(const char *mode_name);

/**
 * Send a command to set the flight mode using COMMAND_LONG
 * This function sends a MAVLink COMMAND_LONG message with MAV_CMD_DO_SET_MODE
 * which will generate a command acknowledgment
 * 
 * @param mode_name Flight mode name (e.g., "GUIDED", "FOLLOW", "AUTO", "LOITER")
 */
void send_set_mode_command(const char *mode_name);

/**
 * Send current GPS coordinates to the drone
 * This function sends a MAVLink GLOBAL_POSITION_INT message
 * containing the current GPS coordinates and altitude
 * 
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param relative_alt Relative altitude in meters
 */
void sendGPSCoordinates(double lat, double lon, float alt, float relative_alt);

/**
 * Send position target to the drone
 * This function sends a MAVLink POSITION_TARGET_GLOBAL_INT message
 * containing the current GPS coordinates and velocity
 * 
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param vx Velocity in X direction (m/s)
 * @param vy Velocity in Y direction (m/s)
 * @param vz Velocity in Z direction (m/s)
 */
// void sendPositionTarget(double lat, double lon, float alt, float vx, float vy, float vz);

/**
 * Send position target to the drone (simplified version)
 * This function sends a MAVLink SET_POSITION_TARGET_GLOBAL_INT message
 * containing only position data (no velocity)
 * 
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 */
void send_position_target(float lat, float lon, float alt);

/**
 * Send attitude target to the drone (yaw control only)
 * This function sends a MAVLink SET_ATTITUDE_TARGET message
 * containing only yaw control (no position or other attitude)
 * 
 * @param yaw_rad Yaw angle in radians
 */
void send_attitude_target_yaw(float yaw_rad);

/**
 * Send a FOLLOW_TARGET message to the drone with custom capabilities
 * This function sends a MAVLink FOLLOW_TARGET message to the autopilot
 * 
 * @param timestamp Timestamp in milliseconds
 * @param lat Latitude of target in degrees
 * @param lon Longitude of target in degrees
 * @param alt Altitude of target in meters
 * @param capabilities Estimated capabilities bitmask
 */
void sendFollowTargetLatLonWithCapabilities(uint64_t timestamp, double lat, double lon, float alt, uint8_t capabilities);

/**
 * Send mission item to the autopilot
 * This function sends a MAVLink MISSION_ITEM message
 * 
 * @param seq Sequence number
 * @param frame Coordinate frame
 * @param command MAVLink command
 * @param autocontinue Auto continue flag
 * @param param1 Parameter 1
 * @param param2 Parameter 2
 * @param param3 Parameter 3
 * @param param4 Parameter 4
 * @param x X coordinate (latitude for global frame)
 * @param y Y coordinate (longitude for global frame)
 * @param z Z coordinate (altitude)
 */
void sendMissionItem(uint16_t seq, MAV_FRAME frame, uint16_t command, uint8_t autocontinue,
                    float param1, float param2, float param3, float param4,
                    float x, float y, float z);

/**
 * Send mission count to the autopilot
 * This function sends a MAVLink MISSION_COUNT message
 * 
 * @param count Number of mission items
 */
void sendMissionCount(uint16_t count);

/**
 * Send mission request to the autopilot
 * This function sends a MAVLink MISSION_REQUEST message
 * 
 * @param seq Sequence number of requested mission item
 */
void sendMissionRequest(uint16_t seq);

/**
 * Send mission ack to the autopilot
 * This function sends a MAVLink MISSION_ACK message
 * 
 * @param type Mission ack type
 */
void sendMissionAck(uint8_t type);

/**
 * Send heartbeat message to the autopilot
 * This function sends a MAVLink HEARTBEAT message to keep the connection alive
 */
void send_heartbeat();

/**
 * Send arm/disarm command to the autopilot
 * This function sends a MAVLink COMMAND_LONG message to arm or disarm the vehicle
 * 
 * @param arm True to arm the vehicle, false to disarm
 * @param force Force arm/disarm (optional, defaults to false)
 */
void send_arm_command(bool arm, bool force = false);

/**
 * Send takeoff command to the autopilot
 * This function sends a MAVLink COMMAND_LONG message to initiate takeoff
 * 
 * @param altitude Target altitude for takeoff in meters
 */
void send_takeoff_command(float altitude);



/**
 * Request radio status messages from the autopilot
 * This function sends a MAVLink COMMAND_LONG message with SET_MESSAGE_INTERVAL
 * to request periodic radio status updates
 * 
 * @param interval_ms Interval between messages in milliseconds (0 = default rate, -1 = disable)
 */
void request_radio_status(int32_t interval_ms = 1000);

/**
 * Request status text messages from the autopilot
 * This function sends a MAVLink COMMAND_LONG message with SET_MESSAGE_INTERVAL
 * to request periodic status text updates
 * 
 * @param interval_ms Interval between messages in milliseconds (0 = default rate, -1 = disable)
 */
void request_status_text(int32_t interval_ms = 1000);

/**
 * Request data stream from the autopilot
 * This function sends a MAVLink REQUEST_DATA_STREAM message to request
 * periodic data stream updates
 * 
 * @param stream_id The data stream ID (e.g., MAV_DATA_STREAM_POSITION)
 * @param message_rate The message rate in Hz (0 = default rate, -1 = disable)
 */
void request_data_stream(uint8_t stream_id, uint16_t message_rate);

/**
 * Send home position to the autopilot
 * This function sends a MAVLink SET_HOME_POSITION message to set the home position
 * 
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 */
void send_home_position(double lat, double lon, float alt);

/**
 * Send status text message to ground station console
 * This function sends a MAVLink STATUSTEXT message that will appear in the ground station console
 * 
 * @param text The text message to send (max 50 characters)
 * @param severity Severity level (MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_ALERT, etc.)
 */
void send_status_text(const char* text, uint8_t severity = MAV_SEVERITY_INFO); 