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
 * Calculate offset coordinates for following behind a target
 * @param target_lat Target latitude in degrees
 * @param target_lon Target longitude in degrees
 * @param offset_distance Distance to offset in meters (positive = behind, negative = in front)
 * @param offset_lat Output: offset latitude in degrees
 * @param offset_lon Output: offset longitude in degrees
 */
void calculateOffsetPosition(double target_lat, double target_lon, 
                           double offset_distance, 
                           double &offset_lat, double &offset_lon) {
    // Convert distance to degrees (approximate)
    // 1 degree of latitude ≈ 111,320 meters
    // 1 degree of longitude ≈ 111,320 * cos(latitude) meters
    
    double lat_offset_deg = offset_distance / 111320.0; // Convert meters to degrees
    
    // Calculate longitude offset (depends on latitude)
    double lon_offset_deg = offset_distance / (111320.0 * cos(target_lat * M_PI / 180.0));
    
    // Calculate offset coordinates
    offset_lat = target_lat - lat_offset_deg; // Behind = subtract latitude
    offset_lon = target_lon - lon_offset_deg; // Behind = subtract longitude
} 