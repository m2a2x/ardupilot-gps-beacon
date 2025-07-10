#pragma once
#include <Arduino.h>

// Battery specifications
extern const float BATTERY_NOMINAL_VOLTAGE;
extern const float BATTERY_MAX_VOLTAGE;
extern const float BATTERY_MIN_VOLTAGE;
extern const float BATTERY_CAPACITY;
extern const float BATTERY_MAX_CURRENT;

// Battery information
extern float battery_voltage;
extern float battery_current;
extern int battery_remaining;

/**
 * Initialize battery with default values
 */
void initBattery();

/**
 * Update battery information from MAVLink system status
 * @param voltage_battery Battery voltage in millivolts
 * @param current_battery Battery current in centiamps
 * @param battery_remaining Battery remaining percentage
 */
void updateBatteryInfo(uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining); 