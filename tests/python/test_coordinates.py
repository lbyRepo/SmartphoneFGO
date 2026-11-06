import math
import unittest

from python_tools.gnss.coordinates import Ecef, Enu, Lla, ecef_to_enu, ecef_to_lla, enu_to_ecef, lla_to_ecef


class CoordinateTransformTest(unittest.TestCase):
    def assert_close(self, actual: float, expected: float, tol: float) -> None:
        self.assertLessEqual(abs(actual - expected), tol, f"{actual} != {expected} within {tol}")

    def test_lla_to_ecef_at_equator_prime_meridian(self) -> None:
        ecef = lla_to_ecef(Lla(0.0, 0.0, 0.0))
        self.assert_close(ecef.x_m, 6378137.0, 1e-6)
        self.assert_close(ecef.y_m, 0.0, 1e-6)
        self.assert_close(ecef.z_m, 0.0, 1e-6)

    def test_lla_to_ecef_at_north_pole(self) -> None:
        ecef = lla_to_ecef(Lla(90.0, 0.0, 0.0))
        self.assert_close(ecef.x_m, 0.0, 1e-6)
        self.assert_close(ecef.y_m, 0.0, 1e-6)
        self.assert_close(ecef.z_m, 6356752.314245179, 1e-3)

    def test_ecef_to_lla_round_trip_singapore(self) -> None:
        original = Lla(1.3521, 103.8198, 15.0)
        round_trip = ecef_to_lla(lla_to_ecef(original))
        self.assert_close(round_trip.lat_deg, original.lat_deg, 1e-9)
        self.assert_close(round_trip.lon_deg, original.lon_deg, 1e-9)
        self.assert_close(round_trip.alt_m, original.alt_m, 1e-5)

    def test_enu_round_trip(self) -> None:
        origin = Lla(1.3521, 103.8198, 15.0)
        offset = Enu(25.0, -12.0, 3.5)
        point = enu_to_ecef(offset, origin)
        round_trip = ecef_to_enu(point, origin)
        self.assert_close(round_trip.east_m, offset.east_m, 1e-8)
        self.assert_close(round_trip.north_m, offset.north_m, 1e-8)
        self.assert_close(round_trip.up_m, offset.up_m, 1e-8)

    def test_ecef_to_enu_origin_is_zero(self) -> None:
        origin = Lla(52.0, 13.0, 100.0)
        enu = ecef_to_enu(lla_to_ecef(origin), origin)
        self.assertLess(math.hypot(enu.east_m, enu.north_m), 1e-9)
        self.assert_close(enu.up_m, 0.0, 1e-9)


if __name__ == "__main__":
    unittest.main()
