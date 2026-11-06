import json
import shutil
import subprocess
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUTPUT_ROOT = ROOT / "tests" / "evaluation_outputs"


class ReplayEvaluationTest(unittest.TestCase):
    def setUp(self) -> None:
        shutil.rmtree(OUTPUT_ROOT, ignore_errors=True)
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    def test_single_command_replays_synthetic_dataset(self) -> None:
        out_dir = OUTPUT_ROOT / "synthetic"
        result = subprocess.run(
            [
                sys.executable,
                "python_tools/replay/run_replay.py",
                "--dataset",
                "synthetic",
                "--output-dir",
                str(out_dir),
            ],
            cwd=ROOT,
            check=True,
            text=True,
            capture_output=True,
        )
        metrics = json.loads((out_dir / "metrics_summary.json").read_text(encoding="utf-8"))
        self.assertEqual(metrics["dataset"], "synthetic")
        self.assertIn("wls", metrics["estimators"])
        self.assertIn("ekf", metrics["estimators"])
        self.assertIn("fgo", metrics["estimators"])
        self.assertIn("android_fused", metrics["baselines"])
        self.assertLess(metrics["estimators"]["wls"]["position_error_rms_m"], 1.0)
        self.assertLess(metrics["estimators"]["fgo"]["position_error_rms_m"], 1.0)
        self.assertGreater(metrics["baselines"]["android_fused"]["position_error_rms_m"], 1.0)
        self.assertEqual(metrics["estimators"]["fgo"]["solver_failures"], 0)
        for name in [
            "truth.csv",
            "android_fused.csv",
            "estimates_wls.csv",
            "estimates_ekf.csv",
            "estimates_fgo.csv",
            "solver_iterations.csv",
            "metrics_summary.json",
            "trajectory_xy.svg",
            "position_error.svg",
        ]:
            self.assertTrue((out_dir / name).is_file(), name)
        self.assertIn('"dataset": "synthetic"', result.stdout)

    def test_single_command_preprocesses_log_dataset(self) -> None:
        out_dir = OUTPUT_ROOT / "log"
        subprocess.run(
            [
                sys.executable,
                "python_tools/replay/run_replay.py",
                "--dataset",
                "log",
                "--input",
                "tests/data/sample_preprocess_log.ndjson",
                "--output-dir",
                str(out_dir),
            ],
            cwd=ROOT,
            check=True,
            text=True,
            capture_output=True,
        )
        metrics = json.loads((out_dir / "metrics_summary.json").read_text(encoding="utf-8"))
        self.assertEqual(metrics["dataset"], "log")
        self.assertEqual(metrics["accepted_observation_count"], 1)
        self.assertEqual(metrics["rejected_measurement_count"], 4)
        self.assertEqual(metrics["positioning_status"], "not_run_no_ephemeris_provider")
        self.assertTrue((out_dir / "observations.csv").is_file())
        self.assertTrue((out_dir / "rejected_measurements.csv").is_file())


if __name__ == "__main__":
    unittest.main()
