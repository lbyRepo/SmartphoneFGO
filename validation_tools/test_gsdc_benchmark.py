import csv
import unittest
from pathlib import Path

import numpy as np

from gsdc_benchmark import (
    check_factor_jacobians,
    compute_metrics,
    detect_derived_schema,
    ecef_delta_to_enu,
    ecef_to_lla,
    lla_to_ecef,
    load_derived_epochs,
    load_ground_truth,
    run_fixed_lag_fgo,
    run_wls,
)


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "datasets" / "gsdc_2023_sample"
SAMSUNG_SEGMENT = ROOT / "datasets" / "gsdc_2021_s20ultra_segment"


class CoordinateTests(unittest.TestCase):
    def test_lla_ecef_round_trip(self) -> None:
        lla = np.array([1.3521, 103.8198, 42.0])
        recovered = ecef_to_lla(lla_to_ecef(*lla))
        np.testing.assert_allclose(recovered, lla, atol=1.0e-7)

    def test_enu_axes(self) -> None:
        reference = lla_to_ecef(0.0, 0.0, 0.0)
        np.testing.assert_allclose(ecef_delta_to_enu(np.array([0.0, 1.0, 0.0]), reference),
                                   [1.0, 0.0, 0.0], atol=1.0e-12)
        np.testing.assert_allclose(ecef_delta_to_enu(np.array([0.0, 0.0, 1.0]), reference),
                                   [0.0, 1.0, 0.0], atol=1.0e-12)


class BenchmarkTests(unittest.TestCase):
    def test_fixture_wls_and_jacobians(self) -> None:
        epochs, _ = load_derived_epochs(FIXTURE / "device_gnss.csv")
        self.assertEqual(len(epochs), 5)
        estimates = run_wls(epochs)
        self.assertTrue(all(estimate.converged for estimate in estimates))
        state = np.zeros(8)
        state[:3] = estimates[0].position_m
        state[6] = estimates[0].clock_bias_m
        errors = check_factor_jacobians(state, epochs[0].observations[0])
        self.assertLess(errors["pseudorange"], 2.0e-5)
        self.assertLess(errors["doppler"], 2.0e-5)

    def test_legacy_samsung_segment_matches_android_regression(self) -> None:
        with (SAMSUNG_SEGMENT / "device_gnss.csv").open() as stream:
            self.assertEqual(
                detect_derived_schema(next(csv.reader(stream))),
                "gsdc_2021_legacy_derived",
            )
        epochs, rejections = load_derived_epochs(SAMSUNG_SEGMENT / "device_gnss.csv")
        truth = load_ground_truth(SAMSUNG_SEGMENT / "ground_truth.csv")
        self.assertEqual(len(epochs), 600)
        self.assertEqual(sum(len(epoch.observations) for epoch in epochs), 15_554)
        self.assertEqual(rejections, {})

        wls = run_wls(epochs)
        fgo = run_fixed_lag_fgo(epochs, wls, lag_seconds=2.0)
        wls_metrics = compute_metrics(wls, truth)
        fgo_metrics = compute_metrics(fgo, truth)
        self.assertEqual(wls_metrics["matched_epochs"], 600)
        self.assertEqual(wls_metrics["solver_failures"], 1)
        self.assertEqual(fgo_metrics["solver_failures"], 3)
        self.assertLess(
            fgo_metrics["trajectory_second_difference_rms_m"],
            wls_metrics["trajectory_second_difference_rms_m"],
        )
        self.assertLess(
            fgo_metrics["horizontal_error_m"]["p95"],
            wls_metrics["horizontal_error_m"]["p95"],
        )


if __name__ == "__main__":
    unittest.main()
