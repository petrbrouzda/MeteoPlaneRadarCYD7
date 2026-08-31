#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Projects a position along a constant great-circle course.
// latitude and longitude are in degrees, speed in knots, bearing in degrees
// clockwise from true north, and seconds is elapsed time.
void Geo_ProjectPosition(double latitude, double longitude,
                         double speedKnots, double bearingDegrees,
                         double seconds, double* resultLatitude,
                         double* resultLongitude);

// Returns the distance in kilometers between two geographic coordinates.
// Input coordinates are in degrees latitude/longitude.
double Geo_DistanceKm(double latitude1, double longitude1,
                      double latitude2, double longitude2);

#ifdef __cplusplus
}
#endif