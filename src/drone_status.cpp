#include "drone_status.h"

DroneStatus::DroneStatus() {
    reset();
}

void DroneStatus::reset() {
    // Position and altitude
    latitude = 0.0;
    longitude = 0.0;
    altitude_amsl = 0.0f;
    altitude_relative = 0.0f;
    altitude_terrain = 0.0f;
    position_valid = false;
    last_position_update = 0;
    
    // Home position
    home_latitude = 0.0;
    home_longitude = 0.0;
    home_altitude_amsl = 0.0f;
    home_position_valid = false;
    last_home_position_update = 0;
    
    // Flight state
    flight_mode = 0;
    is_armed = false;
    is_in_air = false;
    landed_state = MAV_LANDED_STATE_UNDEFINED;
    
    // System status
    gps_fix_type = GPS_FIX_TYPE_NO_GPS;
    satellite_count = 0;
    battery_remaining = 0;
    battery_voltage = 0;
    system_healthy = false;
    
    // Radio status
    radio_rssi = 0;
    
    // Timestamps
    last_heartbeat = 0;
    last_mode_update = 0;
    last_arm_status_update = 0;
    last_battery_update = 0;
    last_gps_update = 0;
    last_radio_update = 0;
    
    // Validity flags
    altitude_valid = false;
    mode_valid = false;
    arm_status_valid = false;
    battery_valid = false;
    gps_valid = false;
    radio_valid = false;
}

// MAVLink message parsing methods
void DroneStatus::parseMessage(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            parseHeartbeat(msg);
            break;
        case MAVLINK_MSG_ID_ALTITUDE:
            parseAltitude(msg);
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            parseGlobalPositionInt(msg);
            break;
        case MAVLINK_MSG_ID_HOME_POSITION:
            parseHomePosition(msg);
            break;
        case MAVLINK_MSG_ID_SYS_STATUS:
            parseSysStatus(msg);
            break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            parseGPSRawInt(msg);
            break;
        case MAVLINK_MSG_ID_RADIO_STATUS:
            parseRadioStatus(msg);
            break;
    }
}

void DroneStatus::parseHeartbeat(const mavlink_message_t& msg) {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    
    // Update heartbeat timestamp
    last_heartbeat = millis();
    
    // Update flight mode
    flight_mode = heartbeat.custom_mode;
    mode_valid = true;
    last_mode_update = millis();
    
    // Update arm status
    is_armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    is_in_air = (heartbeat.system_status == MAV_STATE_ACTIVE);
    
    if (is_in_air) {
        landed_state = MAV_LANDED_STATE_IN_AIR;
    } else if (is_armed) {
        landed_state = MAV_LANDED_STATE_ON_GROUND;
    } else {
        landed_state = MAV_LANDED_STATE_UNDEFINED;
    }
    
    arm_status_valid = true;
    last_arm_status_update = millis();
}

void DroneStatus::parseAltitude(const mavlink_message_t& msg) {
    mavlink_altitude_t altitude_msg;
    mavlink_msg_altitude_decode(&msg, &altitude_msg);
    
    altitude_amsl = altitude_msg.altitude_amsl;
    altitude_relative = altitude_msg.altitude_relative;
    altitude_terrain = altitude_msg.altitude_terrain;
    altitude_valid = true;
    last_position_update = millis();
}

void DroneStatus::parseGlobalPositionInt(const mavlink_message_t& msg) {
    mavlink_global_position_int_t global_pos;
    mavlink_msg_global_position_int_decode(&msg, &global_pos);
    
    latitude = global_pos.lat / 1e7;  // Convert from degE7 to degrees
    longitude = global_pos.lon / 1e7;  // Convert from degE7 to degrees
    altitude_amsl = global_pos.alt / 1000.0f;  // Convert from mm to meters
    altitude_relative = global_pos.relative_alt / 1000.0f;  // Convert from mm to meters
    
    position_valid = true;
    altitude_valid = true;
    last_position_update = millis();
}

void DroneStatus::parseHomePosition(const mavlink_message_t& msg) {
    mavlink_home_position_t home_pos;
    mavlink_msg_home_position_decode(&msg, &home_pos);
    
    home_latitude = home_pos.latitude / 1e7;  // Convert from degE7 to degrees
    home_longitude = home_pos.longitude / 1e7;  // Convert from degE7 to degrees
    home_altitude_amsl = home_pos.altitude / 1000.0f;  // Convert from mm to meters
    
    home_position_valid = true;
    last_home_position_update = millis();
}

void DroneStatus::parseSysStatus(const mavlink_message_t& msg) {
    mavlink_sys_status_t sys_status;
    mavlink_msg_sys_status_decode(&msg, &sys_status);
    
    // Update battery status
    battery_remaining = (sys_status.voltage_battery > 0) ? 
        constrain((sys_status.voltage_battery - 1000) * 100 / (1260 - 1000), 0, 100) : 0;
    battery_voltage = sys_status.voltage_battery / 10; // Convert to 0.1V units
    
    battery_valid = true;
    last_battery_update = millis();
    
    // Update system health
    system_healthy = (sys_status.onboard_control_sensors_health & 
                     sys_status.onboard_control_sensors_enabled) == 
                     sys_status.onboard_control_sensors_enabled;
}

void DroneStatus::parseGPSRawInt(const mavlink_message_t& msg) {
    mavlink_gps_raw_int_t gps_raw;
    mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
    
    gps_fix_type = gps_raw.fix_type;
    satellite_count = gps_raw.satellites_visible;
    gps_valid = true;
    last_gps_update = millis();
}

void DroneStatus::parseRadioStatus(const mavlink_message_t& msg) {
    mavlink_radio_status_t radio_status;
    mavlink_msg_radio_status_decode(&msg, &radio_status);
    
    radio_rssi = radio_status.rssi;
    radio_valid = true;
    last_radio_update = millis();
}

// Utility methods
bool DroneStatus::isConnected() const {
    return (millis() - last_heartbeat) < HEARTBEAT_TIMEOUT_MS;
}

bool DroneStatus::isAltitudeValid() const {
    return altitude_valid && !isAltitudeStale();
}

bool DroneStatus::isAltitudeStale() const {
    return (millis() - last_position_update) > DATA_TIMEOUT_MS;
}

bool DroneStatus::isModeValid() const {
    return mode_valid && !isModeStale();
}

bool DroneStatus::isModeStale() const {
    return (millis() - last_mode_update) > DATA_TIMEOUT_MS;
}

bool DroneStatus::isArmStatusValid() const {
    return arm_status_valid && !isArmStatusStale();
}

bool DroneStatus::isArmStatusStale() const {
    return (millis() - last_arm_status_update) > DATA_TIMEOUT_MS;
}

bool DroneStatus::isBatteryValid() const {
    return battery_valid && !isBatteryStale();
}

bool DroneStatus::isBatteryStale() const {
    return (millis() - last_battery_update) > DATA_TIMEOUT_MS;
}

bool DroneStatus::isGPSValid() const {
    return gps_valid && !isGPSStale();
}

bool DroneStatus::isGPSStale() const {
    return (millis() - last_gps_update) > DATA_TIMEOUT_MS;
}

bool DroneStatus::isPositionValid() const {
    return position_valid && !isPositionStale();
}

bool DroneStatus::isPositionStale() const {
    return (millis() - last_position_update) > DATA_TIMEOUT_MS;
}

bool DroneStatus::isHomePositionValid() const {
    return home_position_valid && !isHomePositionStale();
}

bool DroneStatus::isHomePositionStale() const {
    return (millis() - last_home_position_update) > DATA_TIMEOUT_MS;
}

bool DroneStatus::isRadioValid() const {
    return radio_valid && !isRadioStale();
}

bool DroneStatus::isRadioStale() const {
    return (millis() - last_radio_update) > DATA_TIMEOUT_MS;
}
