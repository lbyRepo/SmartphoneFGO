"""Metrics for estimator comparison outputs."""

from __future__ import annotations

import statistics
from collections import defaultdict
from dataclasses import asdict
from typing import Iterable

from python_tools.evaluation.estimators import Estimate
from python_tools.evaluation.math_utils import Vec3, distance, rms, sub
from python_tools.evaluation.synthetic import SyntheticDataset


def trajectory_smoothness(positions: list[Vec3]) -> float:
    if len(positions) < 3:
        return 0.0
    second_differences = []
    for i in range(1, len(positions) - 1):
        accel_like = (
            positions[i + 1][0] - 2.0 * positions[i][0] + positions[i - 1][0],
            positions[i + 1][1] - 2.0 * positions[i][1] + positions[i - 1][1],
            positions[i + 1][2] - 2.0 * positions[i][2] + positions[i - 1][2],
        )
        second_differences.append(distance(accel_like, (0.0, 0.0, 0.0)))
    return rms(second_differences)


def position_jumps(positions: list[Vec3]) -> tuple[float, int]:
    if len(positions) < 2:
        return 0.0, 0
    jumps = [distance(positions[i], positions[i - 1]) for i in range(1, len(positions))]
    large_jump_count = sum(1 for jump in jumps if jump > 25.0)
    return max(jumps), large_jump_count


def update_rate_hz(times: list[float]) -> float:
    if len(times) < 2:
        return 0.0
    duration = times[-1] - times[0]
    if duration <= 0.0:
        return 0.0
    return (len(times) - 1) / duration


def summarize_synthetic_runs(runs: dict[str, Iterable[Estimate]], dataset: SyntheticDataset) -> dict:
    truth_by_epoch = {index: epoch.truth for index, epoch in enumerate(dataset.epochs)}
    summary: dict[str, dict] = {}
    for name, estimates_iter in runs.items():
        estimates = list(estimates_iter)
        positions = [estimate.position_ecef_m for estimate in estimates]
        times = [estimate.time_seconds for estimate in estimates]
        position_errors = [
            distance(estimate.position_ecef_m, truth_by_epoch[estimate.epoch_index].position_ecef_m)
            for estimate in estimates
        ]
        horizontal_errors = [
            distance(
                (estimate.position_ecef_m[0], estimate.position_ecef_m[1], 0.0),
                (
                    truth_by_epoch[estimate.epoch_index].position_ecef_m[0],
                    truth_by_epoch[estimate.epoch_index].position_ecef_m[1],
                    0.0,
                ),
            )
            for estimate in estimates
        ]
        max_jump, large_jump_count = position_jumps(positions)
        summary[name] = {
            "count": len(estimates),
            "horizontal_error_rms_m": rms(horizontal_errors),
            "horizontal_error_mean_m": statistics.fmean(horizontal_errors) if horizontal_errors else 0.0,
            "position_error_rms_m": rms(position_errors),
            "position_error_max_m": max(position_errors, default=0.0),
            "trajectory_smoothness_m": trajectory_smoothness(positions),
            "solver_latency_mean_ms": statistics.fmean([estimate.latency_ms for estimate in estimates]) if estimates else 0.0,
            "solver_iterations_mean": statistics.fmean([estimate.solver_iterations for estimate in estimates]) if estimates else 0.0,
            "update_rate_hz": update_rate_hz(times),
            "solver_failures": sum(estimate.solver_failures for estimate in estimates),
            "max_position_jump_m": max_jump,
            "large_position_jump_count": large_jump_count,
        }
    return summary


def summarize_fused_baseline(dataset: SyntheticDataset) -> dict:
    errors = [
        distance(point, dataset.epochs[index].truth.position_ecef_m)
        for index, point in enumerate(dataset.fused_positions_ecef_m)
    ]
    horizontal_errors = [
        distance(
            (point[0], point[1], 0.0),
            (
                dataset.epochs[index].truth.position_ecef_m[0],
                dataset.epochs[index].truth.position_ecef_m[1],
                0.0,
            ),
        )
        for index, point in enumerate(dataset.fused_positions_ecef_m)
    ]
    max_jump, large_jump_count = position_jumps(list(dataset.fused_positions_ecef_m))
    times = [epoch.truth.time_seconds for epoch in dataset.epochs]
    return {
        "count": len(dataset.fused_positions_ecef_m),
        "horizontal_error_rms_m": rms(horizontal_errors),
        "horizontal_error_mean_m": statistics.fmean(horizontal_errors) if horizontal_errors else 0.0,
        "position_error_rms_m": rms(errors),
        "position_error_max_m": max(errors, default=0.0),
        "trajectory_smoothness_m": trajectory_smoothness(list(dataset.fused_positions_ecef_m)),
        "update_rate_hz": update_rate_hz(times),
        "max_position_jump_m": max_jump,
        "large_position_jump_count": large_jump_count,
    }


def summarize_rejections(reasons: Iterable[str]) -> dict[str, int]:
    counts: dict[str, int] = defaultdict(int)
    for reason in reasons:
        counts[reason] += 1
    return dict(sorted(counts.items()))


def estimate_to_dict(estimate: Estimate) -> dict:
    payload = asdict(estimate)
    payload["x_m"], payload["y_m"], payload["z_m"] = estimate.position_ecef_m
    payload["vx_mps"], payload["vy_mps"], payload["vz_mps"] = estimate.velocity_ecef_mps
    del payload["position_ecef_m"]
    del payload["velocity_ecef_mps"]
    return payload
