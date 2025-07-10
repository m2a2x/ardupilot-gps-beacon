#ifndef FLIGHT_MODES_H
#define FLIGHT_MODES_H

#include <cstdint>
#include <cstring>

/**
 * ArduPilot Flight Mode Constants
 * https://ardupilot.org/copter/docs/parameters.html#fltmode1
 */
namespace FlightModes {
    // Flight mode values
    constexpr uint32_t STABILIZE = 0;
    constexpr uint32_t ACRO = 1;
    constexpr uint32_t ALTHOLD = 2;
    constexpr uint32_t AUTO = 3;
    constexpr uint32_t GUIDED = 4;
    constexpr uint32_t LOITER = 5;
    constexpr uint32_t RTL = 6;
    constexpr uint32_t CIRCLE = 7;
    constexpr uint32_t LAND = 9;
    constexpr uint32_t DRIFT = 11;
    constexpr uint32_t SPORT = 13;
    constexpr uint32_t FLIP = 14;
    constexpr uint32_t AUTOTUNE = 15;
    constexpr uint32_t POSHOLD = 16;
    constexpr uint32_t BRAKE = 17;
    constexpr uint32_t THROW = 18;
    constexpr uint32_t AVOID_ADSB = 19;
    constexpr uint32_t GUIDED_NOGPS = 20;
    constexpr uint32_t SMART_RTL = 21;
    constexpr uint32_t FLOWHOLD = 22;
    constexpr uint32_t FOLLOW = 23;
    constexpr uint32_t ZIGZAG = 24;
    constexpr uint32_t SYSTEMID = 25;
    constexpr uint32_t HELI_AUTOROTATE = 26;
    constexpr uint32_t AUTO_RTL = 27;
    constexpr uint32_t TURTLE = 28;
}

/**
 * Returns the custom mode value for ArduPilot flight modes
 * @param mode Flight mode name as string
 * @return Custom mode value for MAVLink SET_MODE message
 */
uint32_t get_custom_mode_for(const char *mode);

#endif // FLIGHT_MODES_H 