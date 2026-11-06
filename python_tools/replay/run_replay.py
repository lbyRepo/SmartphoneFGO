#!/usr/bin/env python3
"""Single-command replay and evaluation entry point."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from python_tools.evaluation.estimators import Estimate, run_ekf, run_fgo, run_wls
from python_tools.evaluation.metrics import estimate_to_dict, summarize_fused_baseline, summarize_rejections, summarize_synthetic_runs
from python_tools.evaluation.plotting import write_error_svg, write_trajectory_svg
from python_tools.evaluation.synthetic import SyntheticDataset, make_synthetic_dataset
from python_tools.gnss.preprocess import ObservationEpoch, load_ndjson, preprocess_log


def write_estimates_csv(path: Path, estimates: tuple[Estimate, ...]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "estimator",
        "epoch_index",
        "time_seconds",
        "x_m",
        "y_m",
        "z_m",
        "vx_mps",
        "vy_mps",
        "vz_mps",
        "clock_bias_m",
        "clock_drift_mps",
        "converged",
        "solver_iterations",
        "residual_rms",
        "latency_ms",
        "solver_failures",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for estimate in estimates:
            writer.writerow(estimate_to_dict(estimate))


def write_truth_csv(path: Path, dataset: SyntheticDataset) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["epoch_index", "time_seconds", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps", "clock_bias_m", "clock_drift_mps"],
        )
        writer.writeheader()
        for index, epoch in enumerate(dataset.epochs):
            writer.writerow(
                {
                    "epoch_index": index,
                    "time_seconds": epoch.truth.time_seconds,
                    "x_m": epoch.truth.position_ecef_m[0],
                    "y_m": epoch.truth.position_ecef_m[1],
                    "z_m": epoch.truth.position_ecef_m[2],
                    "vx_mps": epoch.truth.velocity_ecef_mps[0],
                    "vy_mps": epoch.truth.velocity_ecef_mps[1],
                    "vz_mps": epoch.truth.velocity_ecef_mps[2],
                    "clock_bias_m": epoch.truth.clock_bias_m,
                    "clock_drift_mps": epoch.truth.clock_drift_mps,
                }
            )


def write_fused_csv(path: Path, dataset: SyntheticDataset) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["epoch_index", "time_seconds", "x_m", "y_m", "z_m"])
        writer.writeheader()
        for index, point in enumerate(dataset.fused_positions_ecef_m):
            writer.writerow(
                {
                    "epoch_index": index,
                    "time_seconds": dataset.epochs[index].truth.time_seconds,
                    "x_m": point[0],
                    "y_m": point[1],
                    "z_m": point[2],
                }
            )


def write_iterations_csv(path: Path, histories: dict[str, tuple[dict, ...]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        fieldnames = ["estimator", "epoch_index", "iteration", "rms", "max_update"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for estimator, history in histories.items():
            for item in history:
                row = {key: item.get(key, "") for key in fieldnames}
                row["estimator"] = item.get("estimator", estimator)
                writer.writerow(row)


def run_synthetic(output_dir: Path) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset = make_synthetic_dataset()
    wls = run_wls(dataset)
    ekf = run_ekf(dataset)
    fgo = run_fgo(dataset)
    runs = {
        "wls": wls.estimates,
        "ekf": ekf.estimates,
        "fgo": fgo.estimates,
    }

    write_truth_csv(output_dir / "truth.csv", dataset)
    write_fused_csv(output_dir / "android_fused.csv", dataset)
    for name, estimates in runs.items():
        write_estimates_csv(output_dir / f"estimates_{name}.csv", estimates)
    write_iterations_csv(output_dir / "solver_iterations.csv", {"wls": wls.iteration_history, "fgo": fgo.iteration_history})
    write_trajectory_svg(output_dir / "trajectory_xy.svg", runs, dataset)
    write_error_svg(output_dir / "position_error.svg", runs, dataset)

    metrics = {
        "dataset": "synthetic",
        "estimators": summarize_synthetic_runs(runs, dataset),
        "baselines": {
            "android_fused": summarize_fused_baseline(dataset),
        },
        "rejected_measurement_count": 0,
    }
    (output_dir / "metrics_summary.json").write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")
    metrics["artifacts"] = sorted(path.name for path in output_dir.iterdir() if path.is_file())
    (output_dir / "metrics_summary.json").write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")
    return metrics


def write_log_observation_csv(path: Path, epochs: list[ObservationEpoch]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "epoch_elapsed_realtime_nanos",
                "constellation",
                "svid",
                "pseudorange_m",
                "pseudorange_uncertainty_m",
                "pseudorange_rate_mps",
                "pseudorange_rate_uncertainty_mps",
                "cn0_dbhz",
                "carrier_frequency_hz",
            ],
        )
        writer.writeheader()
        for epoch in epochs:
            for obs in epoch.observations:
                writer.writerow(
                    {
                        "epoch_elapsed_realtime_nanos": epoch.epoch_elapsed_realtime_nanos,
                        "constellation": obs.constellation_name,
                        "svid": obs.svid,
                        "pseudorange_m": obs.pseudorange_m,
                        "pseudorange_uncertainty_m": obs.pseudorange_uncertainty_m,
                        "pseudorange_rate_mps": obs.pseudorange_rate_mps,
                        "pseudorange_rate_uncertainty_mps": obs.pseudorange_rate_uncertainty_mps,
                        "cn0_dbhz": obs.cn0_dbhz,
                        "carrier_frequency_hz": obs.carrier_frequency_hz if obs.carrier_frequency_hz is not None else "",
                    }
                )


def write_rejections_csv(path: Path, epochs: list[ObservationEpoch]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["epoch_elapsed_realtime_nanos", "constellation_type", "svid", "reason", "detail"],
        )
        writer.writeheader()
        for epoch in epochs:
            for rejected in epoch.rejected:
                writer.writerow(
                    {
                        "epoch_elapsed_realtime_nanos": rejected.epoch_elapsed_realtime_nanos,
                        "constellation_type": rejected.constellation_type if rejected.constellation_type is not None else "",
                        "svid": rejected.svid if rejected.svid is not None else "",
                        "reason": rejected.reason,
                        "detail": rejected.detail,
                    }
                )


def write_fused_location_csv_from_log(path: Path, records: list[dict]) -> int:
    fused_records = [record for record in records if record.get("record_type") == "fused_location"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        fieldnames = [
            "location_elapsed_realtime_nanos",
            "latitude_deg",
            "longitude_deg",
            "altitude_m",
            "horizontal_accuracy_m",
            "provider",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for record in fused_records:
            writer.writerow({field: record.get(field, "") for field in fieldnames})
    return len(fused_records)


def run_log(input_path: Path, output_dir: Path) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    records = load_ndjson(input_path)
    epochs = preprocess_log(input_path)
    write_log_observation_csv(output_dir / "observations.csv", epochs)
    write_rejections_csv(output_dir / "rejected_measurements.csv", epochs)
    fused_count = write_fused_location_csv_from_log(output_dir / "android_fused.csv", records)
    rejection_reasons = [rejected.reason for epoch in epochs for rejected in epoch.rejected]
    accepted_count = sum(len(epoch.observations) for epoch in epochs)
    rejected_count = sum(len(epoch.rejected) for epoch in epochs)
    metrics = {
        "dataset": "log",
        "input": str(input_path),
        "epoch_count": len(epochs),
        "accepted_observation_count": accepted_count,
        "rejected_measurement_count": rejected_count,
        "rejections_by_reason": summarize_rejections(rejection_reasons),
        "android_fused_location_count": fused_count,
        "positioning_status": "not_run_no_ephemeris_provider",
    }
    (output_dir / "metrics_summary.json").write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")
    metrics["artifacts"] = sorted(path.name for path in output_dir.iterdir() if path.is_file())
    (output_dir / "metrics_summary.json").write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")
    return metrics


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay and evaluate synthetic or Samsung S23+ GNSS logs.")
    parser.add_argument("--dataset", choices=["synthetic", "log"], required=True)
    parser.add_argument("--input", type=Path, help="NDJSON log path for --dataset log")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    if args.dataset == "synthetic":
        metrics = run_synthetic(args.output_dir)
    else:
        if args.input is None:
            parser.error("--input is required for --dataset log")
        metrics = run_log(args.input, args.output_dir)

    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
