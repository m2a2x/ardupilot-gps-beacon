#pragma once
#include <Arduino.h>
#include "mavlink/v2.0/common/mavlink.h"

/**
 * Flight validation component that checks for common issues preventing safe flight
 */
class FlightValidator {
public:
    // Validation error types
    enum ValidationError {
        NO_ERROR = 0,
        GPS_NO_FIX,
        GPS_INSUFFICIENT_SATELLITES,
        GPS_STALE_DATA,
        MODE_CHANGE_FAILED,
        ARM_COMMAND_FAILED,
        TAKEOFF_COMMAND_FAILED,
        POSITION_ESTIMATE_POOR,
        BATTERY_LOW,
        RC_SIGNAL_LOST,
        SENSOR_FAILURE,
        UNKNOWN_ERROR
    };

    /**
     * Initialize the validator
     */
    static void init();

    /**
     * Check GPS-related issues from drone status (informational only)
     * @param gps_fix_type GPS fix type from drone (GPS_FIX_TYPE enum)
     * @return ValidationError code
     */
    static ValidationError checkDroneGPSIssues(uint8_t gps_fix_type);
    
    /**
     * Check local beacon GPS issues (mission critical)
     * @return ValidationError code
     */
    static ValidationError checkBeaconGPSIssues();

    /**
     * Check command acknowledgment errors
     * @param command MAVLink command ID
     * @param result MAVLink result code
     * @return ValidationError code
     */
    static ValidationError checkCommandAckError(uint16_t command, uint8_t result);

    /**
     * Check system status for sensor failures
     * @param onboard_control_sensors_present Bitmap of present sensors
     * @param onboard_control_sensors_enabled Bitmap of enabled sensors
     * @param onboard_control_sensors_health Bitmap of healthy sensors
     * @return ValidationError code
     */
    static ValidationError checkSystemStatus(uint32_t onboard_control_sensors_present, 
                                           uint32_t onboard_control_sensors_enabled,
                                           uint32_t onboard_control_sensors_health);

    /**
     * Get human-readable error description
     * @param error ValidationError code
     * @return Error description string
     */
    static const char* getErrorDescription(ValidationError error);

    /**
     * Get error severity level
     * @param error ValidationError code
     * @return 0=info, 1=warning, 2=critical
     */
    static uint8_t getErrorSeverity(ValidationError error);

private:
    // No private constants needed
}; 