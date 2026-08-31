#include "Geo.h"

#include <math.h>

void Geo_ProjectPosition(double latitude, double longitude,
                         double speedKnots, double bearingDegrees,
                         double seconds, double* resultLatitude,
                         double* resultLongitude) {
  if (!resultLatitude || !resultLongitude) return;

  const double degreesToRadians = 0.017453292519943295;
  const double radiansToDegrees = 57.29577951308232;
  const double earthRadiusNm = 3440.065;

  const double lat1 = latitude * degreesToRadians;
  const double bearing = bearingDegrees * degreesToRadians;
  const double angularDistance = (speedKnots * seconds / earthRadiusNm) / 3600.0;

  const double sinLat1 = sin(lat1);
  const double cosLat1 = cos(lat1);
  const double sinDistance = sin(angularDistance);
  const double cosDistance = cos(angularDistance);
  const double lat2 = asin(sinLat1 * cosDistance +
                           cosLat1 * sinDistance * cos(bearing));
  const double longitudeChange = atan2(sin(bearing) * sinDistance * cosLat1,
                                       cosDistance - sinLat1 * sin(lat2));
  double lon2 = longitude * degreesToRadians + longitudeChange;

  lon2 = fmod(lon2 + 3.0 * 3.14159265358979323846,
              2.0 * 3.14159265358979323846) -
         3.14159265358979323846;

  *resultLatitude = lat2 * radiansToDegrees;
  *resultLongitude = lon2 * radiansToDegrees;
}

double Geo_DistanceKm(double latitude1, double longitude1,
                      double latitude2, double longitude2) {
  const double degreesToRadians = 0.017453292519943295;
  const double earthRadiusKm = 6371.0;

  const double lat1 = latitude1 * degreesToRadians;
  const double lon1 = longitude1 * degreesToRadians;
  const double lat2 = latitude2 * degreesToRadians;
  const double lon2 = longitude2 * degreesToRadians;

  const double deltaLat = lat2 - lat1;
  const double deltaLon = lon2 - lon1;

  const double a = sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
                  cos(lat1) * cos(lat2) *
                  sin(deltaLon / 2.0) * sin(deltaLon / 2.0);

  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return earthRadiusKm * c;
}