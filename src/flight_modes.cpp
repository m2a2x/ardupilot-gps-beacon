#include "flight_modes.h"

/**
 * Returns the custom mode value for ArduPilot flight modes
 * @param mode Flight mode name as string
 * @return Custom mode value for MAVLink SET_MODE message
 */
uint32_t get_custom_mode_for(const char *mode)
{
  if (strcmp(mode, "STABILIZE") == 0)
    return FlightModes::STABILIZE;
  if (strcmp(mode, "ACRO") == 0)
    return FlightModes::ACRO;
  if (strcmp(mode, "ALTHOLD") == 0)
    return FlightModes::ALTHOLD;
  if (strcmp(mode, "AUTO") == 0)
    return FlightModes::AUTO;
  if (strcmp(mode, "GUIDED") == 0)
    return FlightModes::GUIDED;
  if (strcmp(mode, "LOITER") == 0)
    return FlightModes::LOITER;
  if (strcmp(mode, "RTL") == 0)
    return FlightModes::RTL;
  if (strcmp(mode, "CIRCLE") == 0)
    return FlightModes::CIRCLE;
  if (strcmp(mode, "LAND") == 0)
    return FlightModes::LAND;
  if (strcmp(mode, "DRIFT") == 0)
    return FlightModes::DRIFT;
  if (strcmp(mode, "SPORT") == 0)
    return FlightModes::SPORT;
  if (strcmp(mode, "FLIP") == 0)
    return FlightModes::FLIP;
  if (strcmp(mode, "AUTOTUNE") == 0)
    return FlightModes::AUTOTUNE;
  if (strcmp(mode, "POSHOLD") == 0)
    return FlightModes::POSHOLD;
  if (strcmp(mode, "BRAKE") == 0)
    return FlightModes::BRAKE;
  if (strcmp(mode, "THROW") == 0)
    return FlightModes::THROW;
  if (strcmp(mode, "AVOID_ADSB") == 0)
    return FlightModes::AVOID_ADSB;
  if (strcmp(mode, "GUIDED_NOGPS") == 0)
    return FlightModes::GUIDED_NOGPS;
  if (strcmp(mode, "SMART_RTL") == 0)
    return FlightModes::SMART_RTL;
  if (strcmp(mode, "FLOWHOLD") == 0)
    return FlightModes::FLOWHOLD;
  if (strcmp(mode, "FOLLOW") == 0)
    return FlightModes::FOLLOW;
  if (strcmp(mode, "ZIGZAG") == 0)
    return FlightModes::ZIGZAG;
  if (strcmp(mode, "SYSTEMID") == 0)
    return FlightModes::SYSTEMID;
  if (strcmp(mode, "HELI_AUTOROTATE") == 0)
    return FlightModes::HELI_AUTOROTATE;
  if (strcmp(mode, "AUTO_RTL") == 0)
    return FlightModes::AUTO_RTL;
  if (strcmp(mode, "TURTLE") == 0)
    return FlightModes::TURTLE;
  return FlightModes::STABILIZE; // Default to STABILIZE
} 