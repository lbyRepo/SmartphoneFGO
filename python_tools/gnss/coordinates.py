"""WGS-84 coordinate transforms used by GNSS preprocessing and tests."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, radians, sin, sqrt

WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass(frozen=True)
class Lla:
    lat_deg: float
    lon_deg: float
    alt_m: float


@dataclass(frozen=True)
class Ecef:
    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True)
class Enu:
    east_m: float
    north_m: float
    up_m: float


def lla_to_ecef(lla: Lla) -> Ecef:
    lat = radians(lla.lat_deg)
    lon = radians(lla.lon_deg)
    sin_lat = sin(lat)
    cos_lat = cos(lat)
    prime_vertical_radius = WGS84_A_M / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (prime_vertical_radius + lla.alt_m) * cos_lat * cos(lon)
    y = (prime_vertical_radius + lla.alt_m) * cos_lat * sin(lon)
    z = (prime_vertical_radius * (1.0 - WGS84_E2) + lla.alt_m) * sin_lat
    return Ecef(x, y, z)


def ecef_to_lla(ecef: Ecef) -> Lla:
    # Bowring-style iteration; stable for normal navigation and test fixtures.
    lon = atan2(ecef.y_m, ecef.x_m)
    p = hypot(ecef.x_m, ecef.y_m)
    lat = atan2(ecef.z_m, p * (1.0 - WGS84_E2))
    alt = 0.0

    for _ in range(8):
        sin_lat = sin(lat)
        n = WGS84_A_M / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        alt = p / max(cos(lat), 1e-15) - n
        lat = atan2(ecef.z_m, p * (1.0 - WGS84_E2 * n / (n + alt)))

    sin_lat = sin(lat)
    n = WGS84_A_M / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / max(cos(lat), 1e-15) - n
    return Lla(degrees(lat), degrees(lon), alt)


def ecef_to_enu(point: Ecef, origin: Lla) -> Enu:
    origin_ecef = lla_to_ecef(origin)
    dx = point.x_m - origin_ecef.x_m
    dy = point.y_m - origin_ecef.y_m
    dz = point.z_m - origin_ecef.z_m

    lat = radians(origin.lat_deg)
    lon = radians(origin.lon_deg)
    sin_lat = sin(lat)
    cos_lat = cos(lat)
    sin_lon = sin(lon)
    cos_lon = cos(lon)

    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return Enu(east, north, up)


def enu_to_ecef(offset: Enu, origin: Lla) -> Ecef:
    origin_ecef = lla_to_ecef(origin)
    lat = radians(origin.lat_deg)
    lon = radians(origin.lon_deg)
    sin_lat = sin(lat)
    cos_lat = cos(lat)
    sin_lon = sin(lon)
    cos_lon = cos(lon)

    dx = -sin_lon * offset.east_m - sin_lat * cos_lon * offset.north_m + cos_lat * cos_lon * offset.up_m
    dy = cos_lon * offset.east_m - sin_lat * sin_lon * offset.north_m + cos_lat * sin_lon * offset.up_m
    dz = cos_lat * offset.north_m + sin_lat * offset.up_m
    return Ecef(origin_ecef.x_m + dx, origin_ecef.y_m + dy, origin_ecef.z_m + dz)
