#include "mavlink_cmds.h"
#include "conf.h"
#include "log_proxy.h"  // For logging
#include "radio.h"
#include "flight_modes.h"

/**
 * Helper function to send MAVLink message through radio
 */
static void sendMavlinkMessage(const mavlink_message_t* msg) {
    radio.writeMessage(msg);
}



/**
 * Send a SET_MODE command to the autopilot
 * This function creates and sends a MAVLink SET_MODE message
 * to change the flight mode of the drone
 * 
 * @param base_mode Base mode flags (e.g., MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
 * @param custom_mode Custom mode number (specific to ArduPilot)
 */
void send_set_mode(const char *mode_name) {
  mavlink_message_t msg;
  
  uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  uint32_t custom_mode = get_custom_mode_for(mode_name);

  // Pack the SET_MODE message
  mavlink_msg_set_mode_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    MAVLINK_TARGET_SYSTEM_ID,    // target_system
    base_mode,  // base mode flags
    custom_mode // custom mode number
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
}

/**
 * Send a command to set the flight mode using COMMAND_LONG
 * This function sends a MAVLink COMMAND_LONG message with MAV_CMD_DO_SET_MODE
 * which will generate a command acknowledgment
 * 
 * @param mode_name Flight mode name (e.g., "GUIDED", "FOLLOW", "AUTO", "LOITER")
 */
void send_set_mode_command(const char *mode_name) {
  mavlink_message_t msg;
  
  uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  uint32_t custom_mode = get_custom_mode_for(mode_name);

  // Pack the COMMAND_LONG message with DO_SET_MODE
  mavlink_msg_command_long_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    MAVLINK_TARGET_SYSTEM_ID,    // target_system
    MAVLINK_TARGET_COMPONENT_ID,  // target_component
    MAV_CMD_DO_SET_MODE,         // command
    0,                           // confirmation
    base_mode,                   // param1: base mode flags
    custom_mode,                 // param2: custom mode number
    0,                           // param3: unused
    0,                           // param4: unused
    0,                           // param5: unused
    0,                           // param6: unused
    0                            // param7: unused
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
  
  LogProxy::log("Sent mode change command: " + String(mode_name));
}


/**
 * Send a FOLLOW_TARGET message to the drone (Lat/Lon only)
 * Simplified version for when only 2D position data is available
 *
 * @param timestamp Timestamp in milliseconds
 * @param lat Latitude of target in degrees
 * @param lon Longitude of target in degrees
 * @param alt Altitude in meters
 */
void sendFollowTargetLatLon(uint64_t timestamp, double lat, double lon, float alt)
{
  mavlink_message_t msg;

  // Convert degrees to microdegrees (multiply by 1e7)
  int32_t lat_int = static_cast<int32_t>(lat * 1e7);
  int32_t lon_int = static_cast<int32_t>(lon * 1e7);
  
  // Only position data is available
  uint8_t est_capabilities = 1;

  // Initialize unused arrays with zeros
  float vel[3] = {0, 0, 0};
  float acc[3] = {0, 0, 0};
  float attitude_q[4] = {0, 0, 0, 0};
  float rates[3] = {0, 0, 0};
  float position_cov[21] = {0};
  uint8_t custom_state = 0;

  // Pack the FOLLOW_TARGET message
  mavlink_msg_follow_target_pack(
      MAVLINK_SYSTEM_ID,   // system_id
      MAVLINK_COMPONENT_ID, // component_id
      &msg,
      timestamp,        // timestamp
      est_capabilities, // estimated capabilities (position only)
      lat_int,          // latitude in microdegrees
      lon_int,          // longitude in microdegrees
      alt,        // altitude in meters (default 0)
      vel,              // velocity (unused)
      acc,              // acceleration (unused)
      attitude_q,       // attitude quaternion (unused)
      rates,            // angular rates (unused)
      position_cov,     // position covariance (unused)
      custom_state      // custom state (unused)
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
}


/**
 * Send mission item to the autopilot
 */
void sendMissionItem(uint16_t seq, MAV_FRAME frame, uint16_t command, uint8_t autocontinue,
                    float param1, float param2, float param3, float param4,
                    float x, float y, float z) {
  mavlink_message_t msg;

  // Pack the MISSION_ITEM message
  mavlink_msg_mission_item_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    MAVLINK_TARGET_SYSTEM_ID,    // target_system
    MAVLINK_TARGET_COMPONENT_ID,    // target_component
    seq,  // sequence number
    frame, // coordinate frame
    command, // command
    0,    // current (0 for false, 1 for true)
    autocontinue, // auto continue
    param1,   // parameter 1
    param2,   // parameter 2
    param3,   // parameter 3
    param4,   // parameter 4
    x,        // x coordinate
    y,        // y coordinate
    z,        // z coordinate
    0         // mission_type (0 for mission item)
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
}

/**
 * Send mission count to the autopilot
 */
void sendMissionCount(uint16_t count) {
  mavlink_message_t msg;

  // Pack the MISSION_COUNT message
  mavlink_msg_mission_count_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    MAVLINK_TARGET_SYSTEM_ID,    // target_system
    MAVLINK_TARGET_COMPONENT_ID,    // target_component
    count, // count
    0,     // mission_type (0 for mission)
    0      // opaque_id (0 for upload to vehicle)
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
}

/**
 * Send mission request to the autopilot
 */
void sendMissionRequest(uint16_t seq) {
  mavlink_message_t msg;

  // Pack the MISSION_REQUEST message
  mavlink_msg_mission_request_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    MAVLINK_TARGET_SYSTEM_ID,    // target_system
    MAVLINK_TARGET_COMPONENT_ID,    // target_component
    seq,  // sequence number
    0     // mission_type (0 for mission)
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
}

/**
 * Send mission ack to the autopilot
 */
void sendMissionAck(uint8_t type) {
  mavlink_message_t msg;

  // Pack the MISSION_ACK message
  mavlink_msg_mission_ack_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    MAVLINK_TARGET_SYSTEM_ID,    // target_system
    MAVLINK_TARGET_COMPONENT_ID,    // target_component
    type, // type
    0,    // mission_type (0 for mission)
    0     // opaque_id (0 for upload to vehicle)
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
}

void send_heartbeat()
{
  mavlink_message_t msg;

  mavlink_msg_heartbeat_pack(
      MAVLINK_SYSTEM_ID,   // system id
      MAVLINK_COMPONENT_ID, // component id
      &msg,
      MAV_TYPE_ONBOARD_CONTROLLER,       // type (better for SITL)
      MAV_TYPE_GCS,       // autopilot (ArduPilot)
      0, // base mode
      0,                                 // custom mode
      MAV_STATE_ACTIVE                   // system status
  );

  sendMavlinkMessage(&msg);
}

// Send command to arm the vehicle
void send_arm_command(bool arm, bool force)
{
  mavlink_message_t msg;

  mavlink_msg_command_long_pack(
      MAVLINK_SYSTEM_ID,   // system_id
      MAVLINK_COMPONENT_ID, // component_id
      &msg,
      MAVLINK_TARGET_SYSTEM_ID,                            // target_system
      MAVLINK_TARGET_COMPONENT_ID,                            // target_component
      MAV_CMD_COMPONENT_ARM_DISARM, // command
      0,                            // confirmation
      arm ? 1.0 : 0.0,              // param1 (1=arm, 0=disarm)
      force && arm ? 2989.0 : 21196.0,     // param2 (force)
      0,                            // param3
      0,                            // param4
      0,                            // param5
      0,                            // param6
      0                             // param7
  );

  sendMavlinkMessage(&msg);
}

// Send takeoff command
void send_takeoff_command(float altitude)
{
  mavlink_message_t msg;

  mavlink_msg_command_long_pack(
      MAVLINK_SYSTEM_ID,   // system_id
      MAVLINK_COMPONENT_ID, // component_id
      &msg,
      MAVLINK_TARGET_SYSTEM_ID,                   // target_system
      MAVLINK_TARGET_COMPONENT_ID,                   // target_component
      MAV_CMD_NAV_TAKEOFF, // command
      0,                   // confirmation
      0,                   // param1 (minimum pitch)
      0,                   // param2 (empty)
      0,                   // param3 (empty)
      0,                   // param4 (yaw angle)
      0,                   // param5 (latitude)
      0,                   // param6 (longitude)
      altitude             // param7 (altitude)
  );

  sendMavlinkMessage(&msg);
}




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
// void sendPositionTarget(double lat, double lon, float alt, float vx, float vy, float vz) {
//   mavlink_message_t msg;
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//   // Convert degrees to microdegrees (multiply by 1e7)
//   int32_t lat_int = static_cast<int32_t>(lat * 1e7);
//   int32_t lon_int = static_cast<int32_t>(lon * 1e7);
//   float alt_float = alt;  // Altitude in meters

//   // Pack the POSITION_TARGET_GLOBAL_INT message
//   mavlink_msg_position_target_global_int_pack(
//     MAVLINK_SYSTEM_ID,    // system_id
//     MAVLINK_COMPONENT_ID,  // component_id
//     &msg,
//     millis(),  // time_boot_ms
//     MAV_FRAME_GLOBAL_INT,  // coordinate frame
//     0x0F00,  // typemask: use position and velocity
//     lat_int,   // latitude in microdegrees
//     lon_int,   // longitude in microdegrees
//     alt_float, // altitude in meters
//     vx,        // velocity x in m/s
//     vy,        // velocity y in m/s
//     vz,        // velocity z in m/s
//     0.0f,      // acceleration x (unused)
//     0.0f,      // acceleration y (unused)
//     0.0f,      // acceleration z (unused)
//     0.0f,      // yaw (unused)
//     0.0f       // yaw rate (unused)
//   );

//   // Convert message to buffer and send
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//   mavSerial.write(buf, len);
// }


// Send position target message (equivalent to DroneKit's set_position_target_global_int)
void send_position_target(float lat, float lon, float alt)
{
  mavlink_message_t msg;

  mavlink_msg_set_position_target_global_int_pack(
      MAVLINK_SYSTEM_ID,   // system_id
      MAVLINK_COMPONENT_ID, // component_id
      &msg,
      0,                                 // time_boot_ms (not used)
      MAVLINK_TARGET_SYSTEM_ID,                                 // target_system
      MAVLINK_TARGET_COMPONENT_ID,                                 // target_component
      MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // frame
      0b0000111111111000,                // type_mask (only positions)
      (int32_t)(lat * 1e7),              // lat_int
      (int32_t)(lon * 1e7),              // lon_int
      alt,
      0, 0, 0,                           // vx, vy, vz (not used)
      0, 0, 0,                           // afx, afy, afz (not used)
      0, 0                               // yaw, yaw_rate (not used)
  );

  sendMavlinkMessage(&msg);
}

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
                              double &offset_lat, double &offset_lon) {
  // Convert distance to degrees (approximate)
  // 1 degree of latitude ≈ 111,320 meters
  // 1 degree of longitude ≈ 111,320 * cos(latitude) meters
  
  double lat_offset_deg = offset_distance / 111320.0; // Convert meters to degrees
  
  // Calculate longitude offset (depends on latitude)
  double lon_offset_deg = offset_distance / (111320.0 * cos(target_lat * M_PI / 180.0));
  
  // Calculate offset coordinates
  offset_lat = target_lat - lat_offset_deg; // Behind = subtract latitude
  offset_lon = target_lon - lon_offset_deg; // Behind = subtract longitude
}

/**
 * Request radio status messages from the autopilot
 * This function sends a MAVLink COMMAND_LONG message with SET_MESSAGE_INTERVAL
 * to request periodic radio status updates
 * 
 * @param interval_ms Interval between messages in milliseconds (0 = default rate, -1 = disable)
 */
void request_radio_status(int32_t interval_ms) {
  mavlink_message_t msg;
  
  // Pack the COMMAND_LONG message with SET_MESSAGE_INTERVAL
  mavlink_msg_command_long_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    MAVLINK_TARGET_SYSTEM_ID,    // target_system
    MAVLINK_TARGET_COMPONENT_ID,    // target_component
    MAV_CMD_SET_MESSAGE_INTERVAL,   // command
    0,    // confirmation
    MAVLINK_MSG_ID_RADIO_STATUS,    // param1: message ID
    interval_ms,                    // param2: interval in milliseconds
    0,    // param3: use for index ID if required
    0,    // param4: unused
    0,    // param5: unused
    0,    // param6: unused
    0     // param7: unused
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
  
  LogProxy::log("Requested radio status messages with interval: " + String(interval_ms) + "ms");
}

/**
 * Request status text messages from the autopilot
 * This function sends a MAVLink COMMAND_LONG message with SET_MESSAGE_INTERVAL
 * to request periodic status text updates
 * 
 * @param interval_ms Interval between messages in milliseconds (0 = default rate, -1 = disable)
 */
void request_status_text(int32_t interval_ms) {
  mavlink_message_t msg;
  
  // Pack the COMMAND_LONG message with SET_MESSAGE_INTERVAL
  mavlink_msg_command_long_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    MAVLINK_TARGET_SYSTEM_ID,    // target_system
    MAVLINK_TARGET_COMPONENT_ID,    // target_component
    MAV_CMD_SET_MESSAGE_INTERVAL,   // command
    0,    // confirmation
    MAVLINK_MSG_ID_STATUSTEXT,      // param1: message ID
    interval_ms,                    // param2: interval in milliseconds
    0,    // param3: use for index ID if required
    0,    // param4: unused
    0,    // param5: unused
    0,    // param6: unused
    0     // param7: unused
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
  
  LogProxy::log("Requested status text messages with interval: " + String(interval_ms) + "ms");
}