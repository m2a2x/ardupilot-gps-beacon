#include "battery.h"

// Battery information
float battery_voltage = 0.0;
float battery_current = 0.0;
int battery_remaining = 0;

// Panasonic 18650 battery specifications
const float BATTERY_NOMINAL_VOLTAGE = 3.6f;  // Nominal voltage in volts
const float BATTERY_MAX_VOLTAGE = 4.2f;      // Maximum voltage in volts
const float BATTERY_MIN_VOLTAGE = 3.0f;      // Minimum voltage in volts
const float BATTERY_CAPACITY = 3.4f;         // Capacity in Ah (3400mAh)
const float BATTERY_MAX_CURRENT = 6.8f;      // Maximum continuous discharge current in A

void initBattery() {
    battery_voltage = BATTERY_NOMINAL_VOLTAGE;
    battery_current = 0.0f;
    battery_remaining = 100;
}

void updateBatteryInfo(uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining) {
    battery_voltage = voltage_battery / 1000.0f;  // Convert to volts
    battery_current = current_battery / 100.0f;   // Convert to amps
    battery_remaining = battery_remaining;
} 