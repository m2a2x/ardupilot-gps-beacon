#pragma once
#include <Arduino.h>
#include <mavlink/v2.0/common/mavlink.h>

/**
 * DroneStatus class to store and manage all drone state information
 * This centralizes all drone-related data and provides a clean interface
 * for accessing drone status from anywhere in the application
 */
class DroneStatus {
public:
    // Position and altitude data
    double latitude;
    double longitude;
    float altitude_amsl;        // Altitude above mean sea level
    float altitude_relative;    // Altitude relative to home
    float altitude_terrain;     // Altitude above terrain
    bool position_valid;
    unsigned long last_position_update;
    
    // Home position data
    double home_latitude;
    double home_longitude;
    float home_altitude_amsl;   // Home altitude above mean sea level
    bool home_position_valid;
    unsigned long last_home_position_update;
    
    // Flight state
    uint8_t flight_mode;        // Current flight mode
    bool is_armed;
    bool is_in_air;
    uint8_t landed_state;
    
    // System status
    uint8_t gps_fix_type;
    uint8_t satellite_count;
    uint8_t battery_remaining;
    int8_t battery_voltage;
    bool system_healthy;
    
    // Radio status
    int8_t radio_rssi;          // Radio signal strength indicator
    
    // Timestamps
    unsigned long last_heartbeat;
    unsigned long last_mode_update;
    unsigned long last_arm_status_update;
    unsigned long last_battery_update;
    unsigned long last_gps_update;
    unsigned long last_radio_update;
    
    // Data validity flags
    bool altitude_valid;
    bool mode_valid;
    bool arm_status_valid;
    bool battery_valid;
    bool gps_valid;
    bool radio_valid;
    
    // Constants
    static const unsigned long DATA_TIMEOUT_MS = 10000;  // 10 seconds timeout
    static const unsigned long HEARTBEAT_TIMEOUT_MS = 5000;  // 5 seconds timeout

    DroneStatus();
    
    // MAVLink message parsing methods
    void parseMessage(const mavlink_message_t& msg);
    void parseHeartbeat(const mavlink_message_t& msg);
    void parseAltitude(const mavlink_message_t& msg);
    void parseGlobalPositionInt(const mavlink_message_t& msg);
    void parseHomePosition(const mavlink_message_t& msg);
    void parseSysStatus(const mavlink_message_t& msg);
    void parseGPSRawInt(const mavlink_message_t& msg);
    void parseRadioStatus(const mavlink_message_t& msg);
    
    // Utility methods
    void reset();
    bool isConnected() const;
    bool isAltitudeValid() const;
    bool isAltitudeStale() const;
    bool isModeValid() const;
    bool isModeStale() const;
    bool isArmStatusValid() const;
    bool isArmStatusStale() const;
    bool isBatteryValid() const;
    bool isBatteryStale() const;
    bool isGPSValid() const;
    bool isGPSStale() const;
    bool isPositionValid() const;
    bool isPositionStale() const;
    bool isHomePositionValid() const;
    bool isHomePositionStale() const;
    bool isRadioValid() const;
    bool isRadioStale() const;
}; 