#include "gps_utils.h"
#include <math.h>  // For sin, cos, atan2, sqrt functions

/**
 * Calculate distance between two GPS coordinates using Haversine formula
 * @param lat1 First latitude in degrees
 * @param lon1 First longitude in degrees
 * @param lat2 Second latitude in degrees
 * @param lon2 Second longitude in degrees
 * @return Distance in meters
 */
float calculateGPSDistance(double lat1, double lon1, double lat2, double lon2) {
    const double EARTH_RADIUS = 6371000.0; // Earth radius in meters
    
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;
    
    double a = sin(dlat/2) * sin(dlat/2) + cos(lat1_rad) * cos(lat2_rad) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return EARTH_RADIUS * c;
}

/**
 * Calculate bearing (yaw angle) between two GPS coordinates
 * @param lat1 First latitude in degrees
 * @param lon1 First longitude in degrees
 * @param lat2 Second latitude in degrees
 * @param lon2 Second longitude in degrees
 * @return Bearing in radians
 */
float calculateGPSBearing(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    
    double dlon = lon2_rad - lon1_rad;
    
    double y = sin(dlon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon);
    
    return atan2(y, x);
}

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
                           double &offset_lat, double &offset_lon) {
    const double EARTH_RADIUS = 6371000.0; // Earth radius in meters
    
    // Convert target coordinates to radians
    double lat1_rad = target_lat * M_PI / 180.0;
    double lon1_rad = target_lon * M_PI / 180.0;
    
    // Calculate angular distance
    double angular_distance = distance / EARTH_RADIUS;
    
    // Calculate new latitude
    double lat2_rad = asin(sin(lat1_rad) * cos(angular_distance) + 
                          cos(lat1_rad) * sin(angular_distance) * cos(bearing));
    
    // Calculate new longitude
    double lon2_rad = lon1_rad + atan2(sin(bearing) * sin(angular_distance) * cos(lat1_rad),
                                      cos(angular_distance) - sin(lat1_rad) * sin(lat2_rad));
    
    // Convert back to degrees
    offset_lat = lat2_rad * 180.0 / M_PI;
    offset_lon = lon2_rad * 180.0 / M_PI;
} 