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
 * Send a FOLLOW_TARGET message to the drone (Lat/Lon only)
 * Simplified version for when only 2D position data is available
 * This function sends a MAVLink FOLLOW_TARGET message to the autopilot
 * 
 * @param timestamp Timestamp in milliseconds
 * @param lat Latitude of target in degrees
 * @param lon Longitude of target in degrees
 * @param alt Altitude of target in meters
 */
void sendFollowTargetLatLon(uint64_t timestamp, double lat, double lon, float alt);

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
 * Calculate offset coordinates for following behind a target
 * This function calculates GPS coordinates that are offset by a specified distance
 * behind the target position, useful for following missions
 * 
 * @param target_lat Target latitude in degrees
 * @param target_lon Target longitude in degrees
 * @param offset_distance Distance to offset in meters (positive = behind, negative = in front)
 * @param offset_lat Output: offset latitude in degrees
 * @param offset_lon Output: offset longitude in degrees
 */
void calculate_offset_position(double target_lat, double target_lon, 
                              double offset_distance, 
                              double &offset_lat, double &offset_lon);

/**
 * Request radio status messages from the autopilot
 * This function sends a MAVLink COMMAND_LONG message with SET_MESSAGE_INTERVAL
 * to request periodic radio status updates
 * 
 * @param interval_ms Interval between messages in milliseconds (0 = default rate, -1 = disable)
 */
void request_radio_status(int32_t interval_ms = 1000); 