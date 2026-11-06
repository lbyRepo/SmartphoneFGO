import unittest
from pathlib import Path

from python_tools.gnss.preprocess import (
    SPEED_OF_LIGHT_MPS,
    GnssPreprocessConfig,
    build_observation,
    preprocess_log,
)


FIXTURE = Path("tests/data/sample_preprocess_log.ndjson")


class GnssPreprocessTest(unittest.TestCase):
    def test_replayed_log_converts_to_clean_observation_epoch(self) -> None:
        epochs = preprocess_log(FIXTURE)
        self.assertEqual(len(epochs), 1)
        epoch = epochs[0]
        self.assertEqual(len(epoch.observations), 1)
        observation = epoch.observations[0]
        self.assertEqual(observation.constellation_name, "GPS")
        self.assertEqual(observation.svid, 5)
        self.assertAlmostEqual(observation.pseudorange_m, 0.07 * SPEED_OF_LIGHT_MPS, places=3)
        self.assertAlmostEqual(observation.pseudorange_uncertainty_m, 12e-9 * SPEED_OF_LIGHT_MPS, places=6)
        self.assertAlmostEqual(observation.pseudorange_rate_mps, -120.4)

    def test_invalid_measurements_are_rejected_with_reasons(self) -> None:
        epoch = preprocess_log(FIXTURE)[0]
        reasons = {rejected.svid: rejected.reason for rejected in epoch.rejected}
        self.assertEqual(reasons[6], "cn0_below_threshold")
        self.assertEqual(reasons[7], "constellation_rejected")
        self.assertEqual(reasons[8], "frequency_rejected")
        self.assertEqual(reasons[9], "invalid_time_state")

    def test_missing_clock_bias_rejects_pseudorange(self) -> None:
        clock = {"time_nanos": 100070000000, "elapsed_realtime_nanos": 1}
        measurement = {
            "gnss_event_elapsed_realtime_nanos": 1,
            "constellation_type": 1,
            "svid": 1,
            "state": 9,
            "received_sv_time_nanos": 100000000000,
            "received_sv_time_uncertainty_nanos": 12,
            "cn0_dbhz": 35.0,
            "pseudorange_rate_mps": -1.0,
            "pseudorange_rate_uncertainty_mps": 0.1,
            "carrier_frequency_hz": 1575420032.0,
        }
        observation, rejection = build_observation(clock, measurement, GnssPreprocessConfig())
        self.assertIsNone(observation)
        self.assertEqual(rejection.reason, "missing_full_bias_nanos")


if __name__ == "__main__":
    unittest.main()
