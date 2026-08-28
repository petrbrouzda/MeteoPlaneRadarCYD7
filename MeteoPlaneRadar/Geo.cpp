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