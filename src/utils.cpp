#include "utils.h"
#include "conf.h"
#include "gps.h"
#include "mission/mission_followme_complete.h"
#include <mavlink/v2.0/common/mavlink.h>
#include <algorithm>  // For std::remove_if
#include <set>        // For std::set
#include <cstring>    // For strcmp
#include "mission/mission_loiter.h"  // For LoiterMission
#include "menu/flight_modes.h"  // For flight mode functions

// MAVLink message history storage
static std::vector<MavlinkMessageInfo> mavlinkMessageHistory;
static const int MAX_MAVLINK_MESSAGES = 7;
static unsigned long lastDisplayUpdate = 0;  // Track last display update time
static const unsigned long DISPLAY_UPDATE_INTERVAL = 10000;  // 10 seconds in milliseconds
static const unsigned long MESSAGE_CLEANUP_INTERVAL = 10000;  // 10 seconds in milliseconds

// Display cache for showing the same messages for 10 seconds
static std::vector<String> cachedDisplayLines;
static bool hasCachedDisplay = false;

// Mission pointer
Mission* currentMission = nullptr;

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
                      float dt, float &vx, float &vy, float &vz) {
  // Convert to meters (approximate)
  const double EARTH_RADIUS = 6371000.0;  // Earth radius in meters
  double lat1_rad = lat1 * M_PI / 180.0;
  double lon1_rad = lon1 * M_PI / 180.0;
  double lat2_rad = lat2 * M_PI / 180.0;
  double lon2_rad = lon2 * M_PI / 180.0;

  // Calculate distances
  double dx = EARTH_RADIUS * cos(lat1_rad) * (lon2_rad - lon1_rad);
  double dy = EARTH_RADIUS * (lat2_rad - lat1_rad);
  double dz = alt2 - alt1;

  // Calculate velocities
  vx = dx / dt;
  vy = dy / dt;
  vz = dz / dt;
}

/**
 * Parse MAVLink message and extract all fields with their values
 * @param msg MAVLink message to parse
 * @param fields Vector to store parsed field information
 */
void parseMavlinkMessage(const mavlink_message_t& msg, std::vector<MavlinkField>& fields) {
  fields.clear();
  
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(&msg, &heartbeat);
      
      fields.push_back({"type", String(heartbeat.type), "uint8_t"});
      fields.push_back({"autopilot", String(heartbeat.autopilot), "uint8_t"});
      fields.push_back({"base_mode", String(heartbeat.base_mode), "uint8_t"});
      fields.push_back({"custom_mode", String(heartbeat.custom_mode), "uint32_t"});
      fields.push_back({"system_status", String(heartbeat.system_status), "uint8_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t sys_status;
      mavlink_msg_sys_status_decode(&msg, &sys_status);
      
      fields.push_back({"voltage_battery", String(sys_status.voltage_battery), "uint16_t"});
      fields.push_back({"current_battery", String(sys_status.current_battery), "int16_t"});
      fields.push_back({"battery_remaining", String(sys_status.battery_remaining), "int8_t"});
      fields.push_back({"drop_rate_comm", String(sys_status.drop_rate_comm), "uint16_t"});
      fields.push_back({"errors_comm", String(sys_status.errors_comm), "uint16_t"});
      fields.push_back({"errors_count1", String(sys_status.errors_count1), "uint16_t"});
      fields.push_back({"errors_count2", String(sys_status.errors_count2), "uint16_t"});
      fields.push_back({"errors_count3", String(sys_status.errors_count3), "uint16_t"});
      fields.push_back({"errors_count4", String(sys_status.errors_count4), "uint16_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
      mavlink_gps_raw_int_t gps_raw;
      mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
      
      fields.push_back({"time_usec", String(gps_raw.time_usec), "uint64_t"});
      fields.push_back({"lat", String(gps_raw.lat), "int32_t"});
      fields.push_back({"lon", String(gps_raw.lon), "int32_t"});
      fields.push_back({"alt", String(gps_raw.alt), "int32_t"});
      fields.push_back({"eph", String(gps_raw.eph), "uint16_t"});
      fields.push_back({"epv", String(gps_raw.epv), "uint16_t"});
      fields.push_back({"vel", String(gps_raw.vel), "uint16_t"});
      fields.push_back({"cog", String(gps_raw.cog), "uint16_t"});
      fields.push_back({"fix_type", String(gps_raw.fix_type), "uint8_t"});
      fields.push_back({"satellites_visible", String(gps_raw.satellites_visible), "uint8_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t global_pos;
      mavlink_msg_global_position_int_decode(&msg, &global_pos);
      
      fields.push_back({"time_boot_ms", String(global_pos.time_boot_ms), "uint32_t"});
      fields.push_back({"lat", String(global_pos.lat), "int32_t"});
      fields.push_back({"lon", String(global_pos.lon), "int32_t"});
      fields.push_back({"alt", String(global_pos.alt), "int32_t"});
      fields.push_back({"relative_alt", String(global_pos.relative_alt), "int32_t"});
      fields.push_back({"vx", String(global_pos.vx), "int16_t"});
      fields.push_back({"vy", String(global_pos.vy), "int16_t"});
      fields.push_back({"vz", String(global_pos.vz), "int16_t"});
      fields.push_back({"hdg", String(global_pos.hdg), "uint16_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t attitude;
      mavlink_msg_attitude_decode(&msg, &attitude);
      
      fields.push_back({"time_boot_ms", String(attitude.time_boot_ms), "uint32_t"});
      fields.push_back({"roll", String(attitude.roll, 4), "float"});
      fields.push_back({"pitch", String(attitude.pitch, 4), "float"});
      fields.push_back({"yaw", String(attitude.yaw, 4), "float"});
      fields.push_back({"rollspeed", String(attitude.rollspeed, 4), "float"});
      fields.push_back({"pitchspeed", String(attitude.pitchspeed, 4), "float"});
      fields.push_back({"yawspeed", String(attitude.yawspeed, 4), "float"});
      break;
    }
    
    case MAVLINK_MSG_ID_VFR_HUD: {
      mavlink_vfr_hud_t vfr_hud;
      mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
      
      fields.push_back({"airspeed", String(vfr_hud.airspeed, 2), "float"});
      fields.push_back({"groundspeed", String(vfr_hud.groundspeed, 2), "float"});
      fields.push_back({"heading", String(vfr_hud.heading), "int16_t"});
      fields.push_back({"throttle", String(vfr_hud.throttle), "uint16_t"});
      fields.push_back({"alt", String(vfr_hud.alt, 2), "float"});
      fields.push_back({"climb", String(vfr_hud.climb, 2), "float"});
      break;
    }
    
    case MAVLINK_MSG_ID_RADIO_STATUS: {
      mavlink_radio_status_t radio_status;
      mavlink_msg_radio_status_decode(&msg, &radio_status);
      
      fields.push_back({"rssi", String(radio_status.rssi), "uint8_t"});
      fields.push_back({"remrssi", String(radio_status.remrssi), "uint8_t"});
      fields.push_back({"txbuf", String(radio_status.txbuf), "uint8_t"});
      fields.push_back({"noise", String(radio_status.noise), "uint8_t"});
      fields.push_back({"remnoise", String(radio_status.remnoise), "uint8_t"});
      fields.push_back({"rxerrors", String(radio_status.rxerrors), "uint16_t"});
      fields.push_back({"fixed", String(radio_status.fixed), "uint16_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_VIBRATION: {
      mavlink_vibration_t vibration;
      mavlink_msg_vibration_decode(&msg, &vibration);
      
      fields.push_back({"time_usec", String(vibration.time_usec), "uint64_t"});
      fields.push_back({"vibration_x", String(vibration.vibration_x, 4), "float"});
      fields.push_back({"vibration_y", String(vibration.vibration_y, 4), "float"});
      fields.push_back({"vibration_z", String(vibration.vibration_z, 4), "float"});
      fields.push_back({"clipping_0", String(vibration.clipping_0), "uint32_t"});
      fields.push_back({"clipping_1", String(vibration.clipping_1), "uint32_t"});
      fields.push_back({"clipping_2", String(vibration.clipping_2), "uint32_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_SCALED_IMU: {
      mavlink_scaled_imu_t scaled_imu;
      mavlink_msg_scaled_imu_decode(&msg, &scaled_imu);
      
      fields.push_back({"time_boot_ms", String(scaled_imu.time_boot_ms), "uint32_t"});
      fields.push_back({"xacc", String(scaled_imu.xacc), "int16_t"});
      fields.push_back({"yacc", String(scaled_imu.yacc), "int16_t"});
      fields.push_back({"zacc", String(scaled_imu.zacc), "int16_t"});
      fields.push_back({"xgyro", String(scaled_imu.xgyro), "int16_t"});
      fields.push_back({"ygyro", String(scaled_imu.ygyro), "int16_t"});
      fields.push_back({"zgyro", String(scaled_imu.zgyro), "int16_t"});
      fields.push_back({"xmag", String(scaled_imu.xmag), "int16_t"});
      fields.push_back({"ymag", String(scaled_imu.ymag), "int16_t"});
      fields.push_back({"zmag", String(scaled_imu.zmag), "int16_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_RAW_IMU: {
      mavlink_raw_imu_t raw_imu;
      mavlink_msg_raw_imu_decode(&msg, &raw_imu);
      
      fields.push_back({"time_usec", String(raw_imu.time_usec), "uint64_t"});
      fields.push_back({"xacc", String(raw_imu.xacc), "int16_t"});
      fields.push_back({"yacc", String(raw_imu.yacc), "int16_t"});
      fields.push_back({"zacc", String(raw_imu.zacc), "int16_t"});
      fields.push_back({"xgyro", String(raw_imu.xgyro), "int16_t"});
      fields.push_back({"ygyro", String(raw_imu.ygyro), "int16_t"});
      fields.push_back({"zgyro", String(raw_imu.zgyro), "int16_t"});
      fields.push_back({"xmag", String(raw_imu.xmag), "int16_t"});
      fields.push_back({"ymag", String(raw_imu.ymag), "int16_t"});
      fields.push_back({"zmag", String(raw_imu.zmag), "int16_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_COMMAND_LONG: {
      mavlink_command_long_t command_long;
      mavlink_msg_command_long_decode(&msg, &command_long);
      
      fields.push_back({"target_system", String(command_long.target_system), "uint8_t"});
      fields.push_back({"target_component", String(command_long.target_component), "uint8_t"});
      fields.push_back({"command", String(command_long.command), "uint16_t"});
      fields.push_back({"confirmation", String(command_long.confirmation), "uint8_t"});
      fields.push_back({"param1", String(command_long.param1, 4), "float"});
      fields.push_back({"param2", String(command_long.param2, 4), "float"});
      fields.push_back({"param3", String(command_long.param3, 4), "float"});
      fields.push_back({"param4", String(command_long.param4, 4), "float"});
      fields.push_back({"param5", String(command_long.param5, 4), "float"});
      fields.push_back({"param6", String(command_long.param6, 4), "float"});
      fields.push_back({"param7", String(command_long.param7, 4), "float"});
      break;
    }
    
    case MAVLINK_MSG_ID_COMMAND_ACK: {
      mavlink_command_ack_t command_ack;
      mavlink_msg_command_ack_decode(&msg, &command_ack);
      
      fields.push_back({"command", String(command_ack.command), "uint16_t"});
      fields.push_back({"result", String(command_ack.result), "uint8_t"});
      fields.push_back({"progress", String(command_ack.progress), "uint8_t"});
      fields.push_back({"result_param2", String(command_ack.result_param2), "int32_t"});
      fields.push_back({"target_system", String(command_ack.target_system), "uint8_t"});
      fields.push_back({"target_component", String(command_ack.target_component), "uint8_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_MISSION_ITEM: {
      mavlink_mission_item_t mission_item;
      mavlink_msg_mission_item_decode(&msg, &mission_item);
      
      fields.push_back({"target_system", String(mission_item.target_system), "uint8_t"});
      fields.push_back({"target_component", String(mission_item.target_component), "uint8_t"});
      fields.push_back({"seq", String(mission_item.seq), "uint16_t"});
      fields.push_back({"frame", String(mission_item.frame), "uint8_t"});
      fields.push_back({"command", String(mission_item.command), "uint16_t"});
      fields.push_back({"current", String(mission_item.current), "uint8_t"});
      fields.push_back({"autocontinue", String(mission_item.autocontinue), "uint8_t"});
      fields.push_back({"param1", String(mission_item.param1, 4), "float"});
      fields.push_back({"param2", String(mission_item.param2, 4), "float"});
      fields.push_back({"param3", String(mission_item.param3, 4), "float"});
      fields.push_back({"param4", String(mission_item.param4, 4), "float"});
      fields.push_back({"x", String(mission_item.x, 4), "float"});
      fields.push_back({"y", String(mission_item.y, 4), "float"});
      fields.push_back({"z", String(mission_item.z, 4), "float"});
      break;
    }
    
    case MAVLINK_MSG_ID_MISSION_REQUEST: {
      mavlink_mission_request_t mission_request;
      mavlink_msg_mission_request_decode(&msg, &mission_request);
      
      fields.push_back({"target_system", String(mission_request.target_system), "uint8_t"});
      fields.push_back({"target_component", String(mission_request.target_component), "uint8_t"});
      fields.push_back({"seq", String(mission_request.seq), "uint16_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_MISSION_COUNT: {
      mavlink_mission_count_t mission_count;
      mavlink_msg_mission_count_decode(&msg, &mission_count);
      
      fields.push_back({"target_system", String(mission_count.target_system), "uint8_t"});
      fields.push_back({"target_component", String(mission_count.target_component), "uint8_t"});
      fields.push_back({"count", String(mission_count.count), "uint16_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_MISSION_ACK: {
      mavlink_mission_ack_t mission_ack;
      mavlink_msg_mission_ack_decode(&msg, &mission_ack);
      
      fields.push_back({"target_system", String(mission_ack.target_system), "uint8_t"});
      fields.push_back({"target_component", String(mission_ack.target_component), "uint8_t"});
      fields.push_back({"type", String(mission_ack.type), "uint8_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_SET_MODE: {
      mavlink_set_mode_t set_mode;
      mavlink_msg_set_mode_decode(&msg, &set_mode);
      
      fields.push_back({"target_system", String(set_mode.target_system), "uint8_t"});
      fields.push_back({"base_mode", String(set_mode.base_mode), "uint8_t"});
      fields.push_back({"custom_mode", String(set_mode.custom_mode), "uint32_t"});
      break;
    }
    
    case MAVLINK_MSG_ID_FOLLOW_TARGET: {
      mavlink_follow_target_t follow_target;
      mavlink_msg_follow_target_decode(&msg, &follow_target);
      
      fields.push_back({"timestamp", String(follow_target.timestamp), "uint64_t"});
      fields.push_back({"est_capabilities", String(follow_target.est_capabilities), "uint8_t"});
      fields.push_back({"lat", String(follow_target.lat), "int32_t"});
      fields.push_back({"lon", String(follow_target.lon), "int32_t"});
      fields.push_back({"alt", String(follow_target.alt, 2), "float"});
      break;
    }
    
    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: {
      mavlink_position_target_global_int_t pos_target;
      mavlink_msg_position_target_global_int_decode(&msg, &pos_target);
      
      fields.push_back({"time_boot_ms", String(pos_target.time_boot_ms), "uint32_t"});
      fields.push_back({"coordinate_frame", String(pos_target.coordinate_frame), "uint8_t"});
      fields.push_back({"type_mask", String(pos_target.type_mask), "uint16_t"});
      fields.push_back({"lat_int", String(pos_target.lat_int), "int32_t"});
      fields.push_back({"lon_int", String(pos_target.lon_int), "int32_t"});
      fields.push_back({"alt", String(pos_target.alt, 2), "float"});
      fields.push_back({"vx", String(pos_target.vx, 2), "float"});
      fields.push_back({"vy", String(pos_target.vy, 2), "float"});
      fields.push_back({"vz", String(pos_target.vz, 2), "float"});
      fields.push_back({"afx", String(pos_target.afx, 2), "float"});
      fields.push_back({"afy", String(pos_target.afy, 2), "float"});
      fields.push_back({"afz", String(pos_target.afz, 2), "float"});
      fields.push_back({"yaw", String(pos_target.yaw, 4), "float"});
      fields.push_back({"yaw_rate", String(pos_target.yaw_rate, 4), "float"});
      break;
    }
    
    case MAVLINK_MSG_ID_MANUAL_CONTROL: {
      mavlink_manual_control_t manual_control;
      mavlink_msg_manual_control_decode(&msg, &manual_control);
      
      fields.push_back({"target", String(manual_control.target), "uint8_t"});
      fields.push_back({"x", String(manual_control.x), "int16_t"});
      fields.push_back({"y", String(manual_control.y), "int16_t"});
      fields.push_back({"z", String(manual_control.z), "int16_t"});
      fields.push_back({"r", String(manual_control.r), "int16_t"});
      fields.push_back({"buttons", String(manual_control.buttons), "uint16_t"});
      break;
    }
    
    default:
      // For unknown message types, just add basic info
      fields.push_back({"msgid", String(msg.msgid), "uint8_t"});
      fields.push_back({"len", String(msg.len), "uint8_t"});
      fields.push_back({"sysid", String(msg.sysid), "uint8_t"});
      fields.push_back({"compid", String(msg.compid), "uint8_t"});
      break;
  }
}

/**
 * Add a new MAVLink message to the message history
 * @param msgid MAVLink message ID
 * @param system_id Source system ID
 * @param component_id Source component ID
 * @param description Human-readable description of the message
 */
void addMavlinkMessage(uint8_t msgid, uint8_t system_id, uint8_t component_id, const String& description) {
  // Clear old messages (older than 10 seconds)
  unsigned long currentTime = millis();
  mavlinkMessageHistory.erase(
    std::remove_if(mavlinkMessageHistory.begin(), mavlinkMessageHistory.end(),
      [currentTime](const MavlinkMessageInfo& msg) {
        return (currentTime - msg.timestamp) > MESSAGE_CLEANUP_INTERVAL; // 10 seconds
      }),
    mavlinkMessageHistory.end()
  );
  
  MavlinkMessageInfo newMsg;
  newMsg.timestamp = currentTime;
  newMsg.msgid = msgid;
  newMsg.system_id = system_id;
  newMsg.component_id = component_id;
  newMsg.description = description;
  newMsg.is_valid = true;
  
  // Add to the beginning of the vector (most recent first)
  mavlinkMessageHistory.insert(mavlinkMessageHistory.begin(), newMsg);
  
  // Keep only the last MAX_MAVLINK_MESSAGES
  if (mavlinkMessageHistory.size() > MAX_MAVLINK_MESSAGES) {
    mavlinkMessageHistory.resize(MAX_MAVLINK_MESSAGES);
  }
}

/**
 * Add a new MAVLink message to the message history with parsed fields
 * @param msg MAVLink message to add
 */
void addMavlinkMessage(const mavlink_message_t& msg) {
  // Clear old messages (older than 10 seconds)
  unsigned long currentTime = millis();
  mavlinkMessageHistory.erase(
    std::remove_if(mavlinkMessageHistory.begin(), mavlinkMessageHistory.end(),
      [currentTime](const MavlinkMessageInfo& msgInfo) {
        return (currentTime - msgInfo.timestamp) > MESSAGE_CLEANUP_INTERVAL; // 10 seconds
      }),
    mavlinkMessageHistory.end()
  );
  
  MavlinkMessageInfo newMsg;
  newMsg.timestamp = currentTime;
  newMsg.msgid = msg.msgid;
  newMsg.system_id = msg.sysid;
  newMsg.component_id = msg.compid;
  newMsg.description = getMavlinkMessageDescription(msg.msgid);
  newMsg.is_valid = true;
  
  // Parse the message fields
  std::vector<MavlinkField> fields;
  parseMavlinkMessage(msg, fields);
  
  // Convert fields to strings for storage
  for (const auto& field : fields) {
    newMsg.fields.push_back(field.name + "=" + field.value);
  }
  
  // Add to the beginning of the vector (most recent first)
  mavlinkMessageHistory.insert(mavlinkMessageHistory.begin(), newMsg);
  
  // Keep only the last MAX_MAVLINK_MESSAGES
  if (mavlinkMessageHistory.size() > MAX_MAVLINK_MESSAGES) {
    mavlinkMessageHistory.resize(MAX_MAVLINK_MESSAGES);
  }
}

/**
 * Get human-readable description for a MAVLink message ID
 * @param msgid MAVLink message ID
 * @return String description of the message type
 */
String getMavlinkMessageDescription(uint8_t msgid) {
  switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      return "Heartbeat";
    case MAVLINK_MSG_ID_SYS_STATUS:
      return "System Status";
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      return "GPS Raw Int";
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      return "Global Position";
    case MAVLINK_MSG_ID_ATTITUDE:
      return "Attitude";
    case MAVLINK_MSG_ID_VFR_HUD:
      return "VFR HUD";
    case MAVLINK_MSG_ID_RADIO_STATUS:
      return "Radio Status";
    case MAVLINK_MSG_ID_COMMAND_LONG:
      return "Command Long";
    case MAVLINK_MSG_ID_COMMAND_ACK:
      return "Command Ack";
    case MAVLINK_MSG_ID_MISSION_ITEM:
      return "Mission Item";
    case MAVLINK_MSG_ID_MISSION_REQUEST:
      return "Mission Request";
    case MAVLINK_MSG_ID_MISSION_COUNT:
      return "Mission Count";
    case MAVLINK_MSG_ID_MISSION_ACK:
      return "Mission Ack";
    case MAVLINK_MSG_ID_SET_MODE:
      return "Set Mode";
    case MAVLINK_MSG_ID_FOLLOW_TARGET:
      return "Follow Target";
    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
      return "Position Target";
    case MAVLINK_MSG_ID_VIBRATION:
      return "Vibration";
    case MAVLINK_MSG_ID_SCALED_IMU:
      return "Scaled IMU";
    case MAVLINK_MSG_ID_RAW_IMU:
      return "Raw IMU";
    default:
      return "Unknown (" + String(msgid) + ")";
  }
}

/**
 * Get MAVLink message display
 * Shows only RSSI, vibration level, and compass status fields
 * Updates at most once every 10 seconds
 * @param lines Vector to store MAVLink message display lines
 */
void getMavlinkMessagesDisplay(std::vector<String>& lines) {
  unsigned long currentTime = millis();
  
  // Check if enough time has passed since last update
  if (currentTime - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) {
    // Return cached display lines to show the same messages for 10 seconds
    if (hasCachedDisplay) {
      lines = cachedDisplayLines;
    } else {
      // Fallback if no cache exists yet
      lines.push_back("No messages yet");
      lines.push_back("Waiting for data...");
    }
    return;
  }
  
  // Update the last display time
  lastDisplayUpdate = currentTime;
  
  // Clear the lines vector and build new display
  lines.clear();
  
  if (mavlinkMessageHistory.empty()) {
    lines.push_back("No messages yet");
    lines.push_back("Waiting for data...");
  } else {
    // Filter unique messages based on message ID and key fields
    std::vector<MavlinkMessageInfo> uniqueMessages;
    std::set<String> seenMessageKeys;
    
    for (const auto& msg : mavlinkMessageHistory) {
      // Create a unique key for this message type and key fields
      String messageKey = String(msg.msgid) + "_";
      
      // Add key field values to make the key unique
      for (const auto& field : msg.fields) {
        if (field.indexOf("rssi") >= 0 || 
            field.indexOf("vibration") >= 0 || 
            field.indexOf("mag") >= 0 ||
            field.indexOf("heading") >= 0 ||
            field.indexOf("yaw") >= 0 ||
            field.indexOf("hdg") >= 0 ||
            field.indexOf("type") >= 0 ||
            field.indexOf("autopilot") >= 0 ||
            field.indexOf("base_mode") >= 0 ||
            field.indexOf("system_status") >= 0) {
          
          int equalsPos = field.indexOf('=');
          if (equalsPos >= 0) {
            String fieldValue = field.substring(equalsPos + 1);
            messageKey += fieldValue + "_";
          }
        }
      }
      
      // If this is a new unique message, add it
      if (seenMessageKeys.find(messageKey) == seenMessageKeys.end()) {
        seenMessageKeys.insert(messageKey);
        uniqueMessages.push_back(msg);
      }
    }
    
    // Show only the last 5 unique messages (most recent first)
    size_t startIndex = 0;
    size_t endIndex = std::min(uniqueMessages.size(), (size_t)MAX_MAVLINK_MESSAGES);
    
    for (size_t i = startIndex; i < endIndex; i++) {
      const MavlinkMessageInfo& msg = uniqueMessages[i];
      String timeStr = String((millis() - msg.timestamp) / 1000) + "s";
      
      // Get short message name
      String shortName = getShortMessageName(msg.msgid);
      
      // Create compact message line
      String compactLine = shortName + " " + timeStr;
      
      // Add relevant fields to the same line
      bool hasRelevantFields = false;
      String rssiValues = "";
      String vibValues = "";
      String magValues = "";
      String headingValue = "";
      String heartbeatValues = "";
      
      for (const auto& field : msg.fields) {
        // Check if this field contains RSSI, vibration, compass, or heartbeat data
        // RSSI: radio signal strength indicators
        // Vibration: vibration_x, vibration_y, vibration_z
        // Compass: mag (magnetometer), heading, yaw, hdg (heading)
        // Heartbeat: type, autopilot, base_mode, system_status
        if (field.indexOf("rssi") >= 0 || 
            field.indexOf("vibration") >= 0 || 
            field.indexOf("mag") >= 0 ||
            field.indexOf("heading") >= 0 ||
            field.indexOf("yaw") >= 0 ||
            field.indexOf("hdg") >= 0 ||
            field.indexOf("type") >= 0 ||
            field.indexOf("autopilot") >= 0 ||
            field.indexOf("base_mode") >= 0 ||
            field.indexOf("system_status") >= 0) {
          
          // Extract just the value part for compact display
          int equalsPos = field.indexOf('=');
          if (equalsPos >= 0) {
            String fieldName = field.substring(0, equalsPos);
            String fieldValue = field.substring(equalsPos + 1);
            
            // Collect values by type
            if (fieldName.indexOf("rssi") >= 0) {
              if (rssiValues.length() > 0) rssiValues += "/";
              rssiValues += fieldValue;
            } else if (fieldName.indexOf("vibration") >= 0) {
              if (vibValues.length() > 0) vibValues += "/";
              vibValues += fieldValue;
            } else if (fieldName.indexOf("mag") >= 0) {
              if (magValues.length() > 0) magValues += "/";
              magValues += fieldValue;
            } else if (fieldName.indexOf("heading") >= 0 || fieldName.indexOf("hdg") >= 0 || fieldName.indexOf("yaw") >= 0) {
              headingValue = fieldValue;
            } else if (fieldName.indexOf("type") >= 0 || fieldName.indexOf("autopilot") >= 0 || 
                       fieldName.indexOf("base_mode") >= 0 || fieldName.indexOf("system_status") >= 0) {
              if (heartbeatValues.length() > 0) heartbeatValues += "/";
              heartbeatValues += fieldValue;
            }
            hasRelevantFields = true;
          }
        }
      }
      
      // Add collected values to the line
      if (rssiValues.length() > 0) {
        compactLine += " " + rssiValues;
      }
      if (vibValues.length() > 0) {
        compactLine += " v" + vibValues;
      }
      if (magValues.length() > 0) {
        compactLine += " m" + magValues;
      }
      if (headingValue.length() > 0) {
        compactLine += " h" + headingValue;
      }
      if (heartbeatValues.length() > 0) {
        compactLine += " hb" + heartbeatValues;
      }
      
      if (!hasRelevantFields) {
        compactLine += " (no data)";
      }
      
      lines.push_back(compactLine);
    }
  }
  
  // Cache the new display lines
  cachedDisplayLines = lines;
  hasCachedDisplay = true;
}

/**
 * Get short message name for compact display
 * @param msgid MAVLink message ID
 * @return Short name for the message type
 */
String getShortMessageName(uint8_t msgid) {
  switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      return "hb";
    case MAVLINK_MSG_ID_SYS_STATUS:
      return "sys";
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      return "gps";
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      return "pos";
    case MAVLINK_MSG_ID_ATTITUDE:
      return "att";
    case MAVLINK_MSG_ID_VFR_HUD:
      return "hud";
    case MAVLINK_MSG_ID_RADIO_STATUS:
      return "radio";
    case MAVLINK_MSG_ID_COMMAND_LONG:
      return "cmd";
    case MAVLINK_MSG_ID_COMMAND_ACK:
      return "ack";
    case MAVLINK_MSG_ID_MISSION_ITEM:
      return "mis";
    case MAVLINK_MSG_ID_MISSION_REQUEST:
      return "req";
    case MAVLINK_MSG_ID_MISSION_COUNT:
      return "cnt";
    case MAVLINK_MSG_ID_MISSION_ACK:
      return "ack";
    case MAVLINK_MSG_ID_SET_MODE:
      return "mode";
    case MAVLINK_MSG_ID_FOLLOW_TARGET:
      return "follow";
    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
      return "target";
    case MAVLINK_MSG_ID_VIBRATION:
      return "vib";
    case MAVLINK_MSG_ID_SCALED_IMU:
      return "imu";
    case MAVLINK_MSG_ID_RAW_IMU:
      return "raw";
    default:
      return String(msgid);
  }
}

/**
 * Update current mission
 * This function should be called regularly in the main loop
 * to update the currently active mission
 */
void updateCurrentMission() {
  if (currentMission != nullptr) {
    currentMission->update();
  }
}



/**
 * Get mission status display
 * @param lines Vector to store mission display lines
 */
void getMissionStatusDisplay(std::vector<String>& lines) {
  lines.push_back("== MISSION STATUS ==");
  if (currentMission) {
    lines.push_back(String("Active Mission: ") + currentMission->getName());
    lines.push_back(String("Updates: ") + getCurrentMissionUpdateCount());
  } else {
    lines.push_back("No active mission");
  }
  lines.push_back("");
}

void getActiveFlightModeDisplay(std::vector<String> &lines) {
  String activeMode = getActiveFlightMode();
  if (activeMode != "None") {
    lines.push_back(String("Active Mission: ") + activeMode);
    lines.push_back(String("Updates: ") + getCurrentMissionUpdateCount());
  } else {
    lines.push_back("No Active Mission");
  }
}



/**
 * Clear all MAVLink messages from history
 * This function can be called to clear previous messages when rendering
 */
void clearMavlinkMessages() {
  mavlinkMessageHistory.clear();
  cachedDisplayLines.clear();
  hasCachedDisplay = false;
  lastDisplayUpdate = 0;
}

// Helper: Get latest battery info from SYS_STATUS
bool getLatestBatteryInfo(float &voltage_battery, float &current_battery, int &battery_remaining) {
    for (const auto& msg : mavlinkMessageHistory) {
        if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
            for (const auto& field : msg.fields) {
                if (field.startsWith("voltage_battery=")) {
                    voltage_battery = field.substring(16).toFloat() / 1000.0f;
                } else if (field.startsWith("current_battery=")) {
                    current_battery = field.substring(16).toFloat() / 100.0f;
                } else if (field.startsWith("battery_remaining=")) {
                    battery_remaining = field.substring(18).toInt();
                }
            }
            return true;
        }
    }
    return false;
}

// Helper: Get latest flight mode from HEARTBEAT
bool getLatestFlightMode(String &mode) {
    for (const auto& msg : mavlinkMessageHistory) {
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            uint8_t base_mode = 0;
            uint32_t custom_mode = 0;
            for (const auto& field : msg.fields) {
                if (field.startsWith("base_mode=")) {
                    base_mode = field.substring(10).toInt();
                } else if (field.startsWith("custom_mode=")) {
                    custom_mode = field.substring(12).toInt();
                }
            }
            // ArduPilot flight mode mapping (partial, extend as needed)
            switch (custom_mode) {
                case 0: mode = "Stabilize"; break;
                case 1: mode = "Acro"; break;
                case 2: mode = "AltHold"; break;
                case 3: mode = "Auto"; break;
                case 4: mode = "Guided"; break;
                case 5: mode = "Loiter"; break;
                case 6: mode = "RTL"; break;
                case 7: mode = "Circle"; break;
                case 9: mode = "Land"; break;
                case 10: mode = "Drift"; break;
                case 11: mode = "Sport"; break;
                case 13: mode = "Flip"; break;
                case 14: mode = "AutoTune"; break;
                case 15: mode = "Position"; break;
                case 16: mode = "Brake"; break;
                case 17: mode = "Throw"; break;
                case 18: mode = "Avoid ADSB"; break;
                case 19: mode = "Guided No GPS"; break;
                case 20: mode = "Smart RTL"; break;
                default: mode = String("Custom:") + String(custom_mode); break;
            }
            return true;
        }
    }
    return false;
}

/**
 * Get the update count from the currently active mission
 * @return Update count as string, or "0" if no mission is active
 */
String getCurrentMissionUpdateCount() {
    if (currentMission != nullptr) {
        return String(currentMission->getUpdateCount());
    }
    return "0";
}

/**
 * Handle command acknowledgment for the current mission
 * @param command The command that was acknowledged
 * @param result The result of the command
 */
void handleMissionCommandAck(uint16_t command, uint8_t result) {
    if (currentMission != nullptr) {
        // Check if the current mission is a FollowMeCompleteMission using string comparison
        if (strcmp(currentMission->getType(), "FollowMeComplete") == 0) {
            // Cast to FollowMeCompleteMission and call the callback
            FollowMeCompleteMission* autoMission = static_cast<FollowMeCompleteMission*>(currentMission);
            autoMission->onCommandAck(command, result);
        }
    }
} 