#pragma once
#include <Arduino.h>

/**
 * GPS utility functions for distance and bearing calculations
 * These functions provide common GPS-related calculations used across the project
 */

/**
 * Calculate distance between two GPS coordinates using Haversine formula
 * @param lat1 First latitude in degrees
 * @param lon1 First longitude in degrees
 * @param lat2 Second latitude in degrees
 * @param lon2 Second longitude in degrees
 * @return Distance in meters
 */
float calculateGPSDistance(double lat1, double lon1, double lat2, double lon2);

/**
 * Calculate bearing (yaw angle) between two GPS coordinates
 * @param lat1 First latitude in degrees
 * @param lon1 First longitude in degrees
 * @param lat2 Second latitude in degrees
 * @param lon2 Second longitude in degrees
 * @return Bearing in radians
 */
float calculateGPSBearing(double lat1, double lon1, double lat2, double lon2);

/**
 * Calculate offset coordinates from a target point given distance and bearing
 * @param target_lat Target latitude in degrees
 * @param target_lon Target longitude in degrees
 * @param distance Distance to offset in meters (positive = in bearing direction, negative = opposite)
 * @param bearing Bearing angle in radians (0 = North, π/2 = East, π = South, 3π/2 = West)
 * @param offset_lat Output: offset latitude in degrees
 * @param offset_lon Output: offset longitude in degrees
 */
void calculateOffsetPosition(double target_lat, double target_lon, 
                           double distance, double bearing,
                           double &offset_lat, double &offset_lon); 