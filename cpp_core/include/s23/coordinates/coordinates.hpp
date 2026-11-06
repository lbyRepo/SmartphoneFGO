#pragma once

#include <cmath>

#include "s23/core/types.hpp"

namespace s23 {

constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84EccentricitySquared =
    kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr double kPi = 3.141592653589793238462643383279502884;

struct Lla {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

struct Ecef {
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
};

struct Enu {
  double east_m = 0.0;
  double north_m = 0.0;
  double up_m = 0.0;
};

inline double DegToRad(double degrees) { return degrees * kPi / 180.0; }
inline double RadToDeg(double radians) { return radians * 180.0 / kPi; }

inline Ecef LlaToEcef(const Lla& lla) {
  const double lat = DegToRad(lla.latitude_deg);
  const double lon = DegToRad(lla.longitude_deg);
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double n = kWgs84SemiMajorAxisM /
                   std::sqrt(1.0 - kWgs84EccentricitySquared * sin_lat * sin_lat);

  return Ecef{
      (n + lla.altitude_m) * cos_lat * std::cos(lon),
      (n + lla.altitude_m) * cos_lat * std::sin(lon),
      (n * (1.0 - kWgs84EccentricitySquared) + lla.altitude_m) * sin_lat,
  };
}

inline Lla EcefToLla(const Ecef& ecef) {
  const double lon = std::atan2(ecef.y_m, ecef.x_m);
  const double p = std::hypot(ecef.x_m, ecef.y_m);
  double lat = std::atan2(ecef.z_m, p * (1.0 - kWgs84EccentricitySquared));
  double alt = 0.0;

  for (int i = 0; i < 8; ++i) {
    const double sin_lat = std::sin(lat);
    const double n = kWgs84SemiMajorAxisM /
                     std::sqrt(1.0 - kWgs84EccentricitySquared * sin_lat * sin_lat);
    alt = p / std::max(std::cos(lat), 1e-15) - n;
    lat = std::atan2(
        ecef.z_m,
        p * (1.0 - kWgs84EccentricitySquared * n / (n + alt)));
  }

  const double sin_lat = std::sin(lat);
  const double n = kWgs84SemiMajorAxisM /
                   std::sqrt(1.0 - kWgs84EccentricitySquared * sin_lat * sin_lat);
  alt = p / std::max(std::cos(lat), 1e-15) - n;
  return Lla{RadToDeg(lat), RadToDeg(lon), alt};
}

inline Enu EcefToEnu(const Ecef& point, const Lla& origin) {
  const Ecef origin_ecef = LlaToEcef(origin);
  const double dx = point.x_m - origin_ecef.x_m;
  const double dy = point.y_m - origin_ecef.y_m;
  const double dz = point.z_m - origin_ecef.z_m;

  const double lat = DegToRad(origin.latitude_deg);
  const double lon = DegToRad(origin.longitude_deg);
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);

  return Enu{
      -sin_lon * dx + cos_lon * dy,
      -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
      cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,
  };
}

inline Ecef EnuToEcef(const Enu& offset, const Lla& origin) {
  const Ecef origin_ecef = LlaToEcef(origin);
  const double lat = DegToRad(origin.latitude_deg);
  const double lon = DegToRad(origin.longitude_deg);
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);

  const double dx = -sin_lon * offset.east_m -
                    sin_lat * cos_lon * offset.north_m +
                    cos_lat * cos_lon * offset.up_m;
  const double dy = cos_lon * offset.east_m -
                    sin_lat * sin_lon * offset.north_m +
                    cos_lat * sin_lon * offset.up_m;
  const double dz = cos_lat * offset.north_m + sin_lat * offset.up_m;

  return Ecef{origin_ecef.x_m + dx, origin_ecef.y_m + dy, origin_ecef.z_m + dz};
}

}  // namespace s23
