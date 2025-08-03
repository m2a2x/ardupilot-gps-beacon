#include "mavlink_cmds.h"
#include "conf.h"
#include "log_proxy.h"  // For logging
#include "radio.h"
#include "flight_modes.h"
#include "gps.h"  // For GPS functions
#include <math.h>  // For cos, sin functions

/**
 * Helper function to send MAVLink message through radio
 */
static void sendMavlinkMessage(const mavlink_message_t* msg) {
    radio.writeMessage(msg);
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
}

/**
 * Send a FOLLOW_TARGET message to the drone with custom capabilities
 * This function sends a MAVLink FOLLOW_TARGET message to the autopilot
 *
 * @param timestamp Timestamp in milliseconds
 * @param lat Latitude of target in degrees
 * @param lon Longitude of target in degrees
 * @param alt Altitude in meters
 * @param capabilities Estimated capabilities bitmask
 */
void sendFollowTargetLatLonWithCapabilities(uint64_t timestamp, double lat, double lon, float alt, uint8_t capabilities)
{
  mavlink_message_t msg;

  // Convert degrees to microdegrees (multiply by 1e7)
  int32_t lat_int = static_cast<int32_t>(lat * 1e7);
  int32_t lon_int = static_cast<int32_t>(lon * 1e7);

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
      capabilities,     // estimated capabilities (custom)
      lat_int,          // latitude in microdegrees
      lon_int,          // longitude in microdegrees
      alt,        // altitude in meters
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
      MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // frame - relative to home altitude (FIXED)
      0b0000111111111000,                // type_mask (only positions)
      (int32_t)(lat * 1e7),              // lat_int
      (int32_t)(lon * 1e7),              // lon_int
      alt,                               // altitude relative to home position
      0, 0, 0,                           // vx, vy, vz (not used)
      0, 0, 0,                           // afx, afy, afz (not used)
      0, 0                               // yaw, yaw_rate (not used)
  );

  sendMavlinkMessage(&msg);
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
}

/**
 * Request data stream from the autopilot
 * This function sends a MAVLink REQUEST_DATA_STREAM message to request
 * periodic data stream updates
 * 
 * @param stream_id The data stream ID (e.g., MAV_DATA_STREAM_POSITION)
 * @param message_rate The message rate in Hz (0 = default rate, -1 = disable)
 */
void request_data_stream(uint8_t stream_id, uint16_t message_rate) {
  mavlink_message_t msg;
  
  // Pack the REQUEST_DATA_STREAM message
  mavlink_msg_request_data_stream_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    MAVLINK_TARGET_SYSTEM_ID,    // target_system
    MAVLINK_TARGET_COMPONENT_ID,    // target_component
    stream_id,                    // req_stream_id
    message_rate,                 // req_message_rate
    1                            // start_stop (1 = start, 0 = stop)
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
}


/**
 * Send status text message to ground station console
 * This function sends a MAVLink STATUSTEXT message that will appear in the ground station console
 * 
 * @param text The text message to send (max 50 characters)
 * @param severity Severity level (MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_ALERT, etc.)
 */
void send_status_text(const char* text, uint8_t severity) {
  mavlink_message_t msg;
  
  // Create a buffer for the text (STATUSTEXT can hold up to 50 characters)
  char text_buffer[50];
  memset(text_buffer, 0, sizeof(text_buffer));
  
  // Copy text to buffer, ensuring it doesn't exceed 50 characters
  strncpy(text_buffer, text, sizeof(text_buffer) - 1);
  
  // Pack the STATUSTEXT message
  mavlink_msg_statustext_pack(
    MAVLINK_SYSTEM_ID,    // system_id
    MAVLINK_COMPONENT_ID,  // component_id
    &msg,
    severity,             // severity level
    text_buffer,          // text message
    0,                    // id (0 for single message)
    0                     // chunk_seq (0 for single message)
  );

  // Send message through radio
  sendMavlinkMessage(&msg);
}

void send_roi_command(double lat, double lon, float alt) {
  mavlink_message_t msg;

  mavlink_msg_command_long_pack(
      MAVLINK_SYSTEM_ID,   // system_id
      MAVLINK_COMPONENT_ID, // component_id
      &msg,
      MAVLINK_TARGET_SYSTEM_ID,    // target_system
      MAVLINK_TARGET_COMPONENT_ID, // target_component
      MAV_CMD_DO_SET_ROI_LOCATION, // command
      0,                           // confirmation
      0,                           // param1 (unused)
      0,                           // param2 (unused)
      0,                           // param3 (unused)
      0,                           // param4 (unused)
      lat,                         // param5 (latitude)
      lon,                         // param6 (longitude)
      alt                          // param7 (altitude)
  );

  sendMavlinkMessage(&msg);
}

void send_roi_clear_command() {
  mavlink_message_t msg;

  mavlink_msg_command_long_pack(
      MAVLINK_SYSTEM_ID,   // system_id
      MAVLINK_COMPONENT_ID, // component_id
      &msg,
      MAVLINK_TARGET_SYSTEM_ID,    // target_system
      MAVLINK_TARGET_COMPONENT_ID, // target_component
      MAV_CMD_DO_SET_ROI_NONE,     // command
      0,                           // confirmation
      0,                           // param1 (unused)
      0,                           // param2 (unused)
      0,                           // param3 (unused)
      0,                           // param4 (unused)
      0,                           // param5 (unused)
      0,                           // param6 (unused)
      0                            // param7 (unused)
  );

  sendMavlinkMessage(&msg);
}