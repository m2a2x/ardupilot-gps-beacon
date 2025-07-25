#include "flight_validator.h"
#include "gps.h"
#include "log_proxy.h"

void FlightValidator::init() {
    // Nothing to initialize for now
}

FlightValidator::ValidationError FlightValidator::checkDroneGPSIssues(uint8_t gps_fix_type) {
    // Drone GPS validation: informational only
    if (gps_fix_type == GPS_FIX_TYPE_NO_GPS || gps_fix_type == GPS_FIX_TYPE_NO_FIX) {
        return GPS_NO_FIX;
    }
    
    // Any other fix type (2D, 3D, DGPS, RTK, etc.) is considered OK
    return NO_ERROR;
}

FlightValidator::ValidationError FlightValidator::checkBeaconGPSIssues() {
    // Beacon GPS validation: mission critical - just check if we have fix
    if (!gpsHasFix()) {
        return GPS_NO_FIX;
    }
    
    return NO_ERROR;
}

FlightValidator::ValidationError FlightValidator::checkCommandAckError(uint16_t command, uint8_t result) {
    // Check if command was accepted
    if (result == MAV_RESULT_ACCEPTED) {
        return NO_ERROR;
    }
    
    // Map command types to specific errors
    switch (command) {
        case MAV_CMD_DO_SET_MODE:
            return MODE_CHANGE_FAILED;
            
        case MAV_CMD_COMPONENT_ARM_DISARM:
            return ARM_COMMAND_FAILED;
            
        case MAV_CMD_NAV_TAKEOFF:
            return TAKEOFF_COMMAND_FAILED;
            
        default:
            // For other commands, return generic error based on result
            switch (result) {
                case MAV_RESULT_TEMPORARILY_REJECTED:
                case MAV_RESULT_DENIED:
                case MAV_RESULT_UNSUPPORTED:
                case MAV_RESULT_FAILED:
                case MAV_RESULT_CANCELLED:
                case MAV_RESULT_COMMAND_LONG_ONLY:
                case MAV_RESULT_COMMAND_INT_ONLY:
                case MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME:
                    return UNKNOWN_ERROR;
                    
                default:
                    return NO_ERROR;
            }
    }
}

FlightValidator::ValidationError FlightValidator::checkSystemStatus(uint32_t onboard_control_sensors_present, 
                                                                   uint32_t onboard_control_sensors_enabled,
                                                                   uint32_t onboard_control_sensors_health) {
    // Check critical sensors
    uint32_t critical_sensors = MAV_SYS_STATUS_SENSOR_3D_GYRO | 
                               MAV_SYS_STATUS_SENSOR_3D_ACCEL | 
                               MAV_SYS_STATUS_SENSOR_3D_MAG | 
                               MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                               MAV_SYS_STATUS_SENSOR_GPS;
    
    // Check if critical sensors are present and enabled
    if ((onboard_control_sensors_present & critical_sensors) != critical_sensors) {
        return SENSOR_FAILURE;
    }
    
    // Check if critical sensors are healthy
    if ((onboard_control_sensors_health & critical_sensors) != critical_sensors) {
        return SENSOR_FAILURE;
    }
    
    // Check RC receiver
    if (onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_RC_RECEIVER) {
        if (!(onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_RC_RECEIVER) ||
            !(onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_RC_RECEIVER)) {
            return RC_SIGNAL_LOST;
        }
    }
    
    // Check battery (if present)
    if (onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_BATTERY) {
        if (!(onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_BATTERY)) {
            return BATTERY_LOW;
        }
    }
    
    return NO_ERROR;
}

FlightValidator::ValidationError FlightValidator::checkRadioConnectivity(unsigned long last_heartbeat_time, unsigned long heartbeat_timeout_ms) {
    // Check if we've received a heartbeat recently
    if (millis() - last_heartbeat_time > heartbeat_timeout_ms) {
        return RADIO_SIGNAL_LOST;
    }
    
    return NO_ERROR;
}

const char* FlightValidator::getErrorDescription(ValidationError error) {
    switch (error) {
        case NO_ERROR:
            return "No errors";
        case GPS_NO_FIX:
            return "GPS: No position fix";
        case GPS_INSUFFICIENT_SATELLITES:
            return "GPS: Insufficient satellites"; // Kept for compatibility
        case GPS_STALE_DATA:
            return "GPS: Data is stale"; // Kept for compatibility
        case MODE_CHANGE_FAILED:
            return "Mode change command failed";
        case ARM_COMMAND_FAILED:
            return "Arm command failed";
        case TAKEOFF_COMMAND_FAILED:
            return "Takeoff command failed";
        case POSITION_ESTIMATE_POOR:
            return "Position estimate poor";
        case BATTERY_LOW:
            return "Battery low or unhealthy";
        case RC_SIGNAL_LOST:
            return "RC signal lost";
        case RADIO_SIGNAL_LOST:
            return "SiK radio signal lost";
        case SENSOR_FAILURE:
            return "Critical sensor failure";
        case UNKNOWN_ERROR:
            return "Unknown error";
        default:
            return "Invalid error code";
    }
}

uint8_t FlightValidator::getErrorSeverity(ValidationError error) {
    switch (error) {
        case NO_ERROR:
            return 0; // info
        case GPS_INSUFFICIENT_SATELLITES:
        case GPS_STALE_DATA:
            return 1; // warning (kept for compatibility)
        case GPS_NO_FIX:
        case MODE_CHANGE_FAILED:
        case ARM_COMMAND_FAILED:
        case TAKEOFF_COMMAND_FAILED:
        case POSITION_ESTIMATE_POOR:
        case BATTERY_LOW:
        case RC_SIGNAL_LOST:
        case RADIO_SIGNAL_LOST:
        case SENSOR_FAILURE:
        case UNKNOWN_ERROR:
            return 2; // critical
        default:
            return 1; // warning
    }
} 