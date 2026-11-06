"""Dependency-free synthetic WLS, EKF, and fixed-lag FGO runners."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from python_tools.evaluation.math_utils import Vec3, add, distance, dot, normalize, rms, scale, solve_normal_equations, sub
from python_tools.evaluation.synthetic import SyntheticDataset, SyntheticEpoch, SyntheticMeasurement


@dataclass(frozen=True)
class Estimate:
    estimator: str
    epoch_index: int
    time_seconds: float
    position_ecef_m: Vec3
    velocity_ecef_mps: Vec3
    clock_bias_m: float
    clock_drift_mps: float
    converged: bool
    solver_iterations: int
    residual_rms: float
    latency_ms: float
    solver_failures: int = 0


@dataclass(frozen=True)
class SolverRun:
    estimates: tuple[Estimate, ...]
    iteration_history: tuple[dict[str, float | int | str], ...] = field(default_factory=tuple)


def pseudorange_residual(position: Vec3, clock_bias_m: float, measurement: SyntheticMeasurement) -> float:
    return distance(position, measurement.satellite_position_ecef_m) + clock_bias_m - measurement.pseudorange_m


def pseudorange_jacobian(position: Vec3, measurement: SyntheticMeasurement) -> tuple[float, float, float, float]:
    unit = normalize(sub(position, measurement.satellite_position_ecef_m))
    return (unit[0], unit[1], unit[2], 1.0)


def doppler_residual(
    position: Vec3,
    velocity: Vec3,
    clock_drift_mps: float,
    measurement: SyntheticMeasurement,
) -> float:
    los = normalize(sub(position, measurement.satellite_position_ecef_m))
    predicted = dot(los, sub(velocity, measurement.satellite_velocity_ecef_mps)) + clock_drift_mps
    return predicted - measurement.pseudorange_rate_mps


def solve_wls_epoch(
    epoch: SyntheticEpoch,
    initial_position: Vec3,
    initial_clock_bias_m: float = 0.0,
    max_iterations: int = 12,
) -> tuple[Vec3, float, bool, list[dict[str, float | int]], list[float]]:
    position = initial_position
    clock_bias = initial_clock_bias_m
    history: list[dict[str, float | int]] = []
    final_residuals: list[float] = []
    converged = False
    for iteration in range(max_iterations):
        jacobian: list[list[float]] = []
        residuals: list[float] = []
        for measurement in epoch.measurements:
            sigma = measurement.pseudorange_sigma_m
            residuals.append(pseudorange_residual(position, clock_bias, measurement) / sigma)
            jac = pseudorange_jacobian(position, measurement)
            jacobian.append([value / sigma for value in jac])
        update = solve_normal_equations(jacobian, residuals, damping=1e-6)
        position = (position[0] + update[0], position[1] + update[1], position[2] + update[2])
        clock_bias += update[3]
        max_update = max(abs(value) for value in update)
        history.append({"iteration": iteration, "rms": rms(residuals), "max_update": max_update})
        final_residuals = residuals
        if max_update < 1e-4:
            converged = True
            break
    return position, clock_bias, converged, history, final_residuals


def run_wls(dataset: SyntheticDataset) -> SolverRun:
    estimates: list[Estimate] = []
    history: list[dict[str, float | int | str]] = []
    for epoch_index, epoch in enumerate(dataset.epochs):
        start = time.perf_counter()
        initial = (
            epoch.truth.position_ecef_m[0] - 250.0,
            epoch.truth.position_ecef_m[1] + 180.0,
            epoch.truth.position_ecef_m[2] - 90.0,
        )
        try:
            position, clock_bias, converged, epoch_history, residuals = solve_wls_epoch(epoch, initial)
            failure = 0 if converged else 1
        except Exception:
            position, clock_bias, converged, epoch_history, residuals, failure = initial, 0.0, False, [], [], 1
        latency_ms = (time.perf_counter() - start) * 1000.0
        estimates.append(
            Estimate(
                "wls",
                epoch_index,
                epoch.truth.time_seconds,
                position,
                (0.0, 0.0, 0.0),
                clock_bias,
                0.0,
                converged,
                len(epoch_history),
                rms(residuals),
                latency_ms,
                failure,
            )
        )
        for item in epoch_history:
            history.append({"estimator": "wls", "epoch_index": epoch_index, **item})
    return SolverRun(tuple(estimates), tuple(history))


def run_ekf(dataset: SyntheticDataset) -> SolverRun:
    # Minimal deterministic baseline: use WLS as GNSS position updates and constant-velocity propagation between epochs.
    wls = run_wls(dataset)
    estimates: list[Estimate] = []
    previous_position: Vec3 | None = None
    previous_time: float | None = None
    velocity: Vec3 = (0.0, 0.0, 0.0)
    for estimate in wls.estimates:
        start = time.perf_counter()
        if previous_position is not None and previous_time is not None:
            dt = max(estimate.time_seconds - previous_time, 1e-9)
            propagated = add(previous_position, scale(velocity, dt))
            corrected = (
                0.5 * propagated[0] + 0.5 * estimate.position_ecef_m[0],
                0.5 * propagated[1] + 0.5 * estimate.position_ecef_m[1],
                0.5 * propagated[2] + 0.5 * estimate.position_ecef_m[2],
            )
            velocity = scale(sub(corrected, previous_position), 1.0 / dt)
            position = corrected
        else:
            position = estimate.position_ecef_m
        previous_position = position
        previous_time = estimate.time_seconds
        latency_ms = estimate.latency_ms + (time.perf_counter() - start) * 1000.0
        estimates.append(
            Estimate(
                "ekf",
                estimate.epoch_index,
                estimate.time_seconds,
                position,
                velocity,
                estimate.clock_bias_m,
                0.0,
                estimate.converged,
                estimate.solver_iterations,
                estimate.residual_rms,
                latency_ms,
                estimate.solver_failures,
            )
        )
    return SolverRun(tuple(estimates), tuple())


def _read_state(parameters: list[float], epoch_index: int) -> tuple[Vec3, Vec3, float, float]:
    offset = epoch_index * 8
    return (
        (parameters[offset], parameters[offset + 1], parameters[offset + 2]),
        (parameters[offset + 3], parameters[offset + 4], parameters[offset + 5]),
        parameters[offset + 6],
        parameters[offset + 7],
    )


def _write_state(parameters: list[float], epoch_index: int, position: Vec3, velocity: Vec3, clock_bias: float, clock_drift: float) -> None:
    offset = epoch_index * 8
    parameters[offset : offset + 8] = [
        position[0],
        position[1],
        position[2],
        velocity[0],
        velocity[1],
        velocity[2],
        clock_bias,
        clock_drift,
    ]


def _add_row(
    residuals: list[float],
    jacobian: list[list[float]],
    residual: float,
    sigma: float,
    terms: list[tuple[int, float]],
    parameter_count: int,
) -> None:
    residuals.append(residual / sigma)
    row = [0.0] * parameter_count
    for index, value in terms:
        row[index] = value / sigma
    jacobian.append(row)


def _build_fgo_system(dataset: SyntheticDataset, parameters: list[float], use_doppler: bool = True) -> tuple[list[list[float]], list[float]]:
    parameter_count = len(parameters)
    jacobian: list[list[float]] = []
    residuals: list[float] = []
    first_position, first_velocity, first_clock_bias, first_clock_drift = _read_state(parameters, 0)
    first_truth = dataset.epochs[0].truth
    for axis in range(3):
        _add_row(residuals, jacobian, first_position[axis] - first_truth.position_ecef_m[axis], 20.0, [(axis, 1.0)], parameter_count)
        _add_row(residuals, jacobian, first_velocity[axis] - first_truth.velocity_ecef_mps[axis], 5.0, [(3 + axis, 1.0)], parameter_count)
    _add_row(residuals, jacobian, first_clock_bias - first_truth.clock_bias_m, 20.0, [(6, 1.0)], parameter_count)
    _add_row(residuals, jacobian, first_clock_drift - first_truth.clock_drift_mps, 5.0, [(7, 1.0)], parameter_count)

    for epoch_index, epoch in enumerate(dataset.epochs):
        position, velocity, clock_bias, clock_drift = _read_state(parameters, epoch_index)
        offset = epoch_index * 8
        for measurement in epoch.measurements:
            pr_res = pseudorange_residual(position, clock_bias, measurement)
            pr_jac = pseudorange_jacobian(position, measurement)
            _add_row(
                residuals,
                jacobian,
                pr_res,
                measurement.pseudorange_sigma_m,
                [(offset + 0, pr_jac[0]), (offset + 1, pr_jac[1]), (offset + 2, pr_jac[2]), (offset + 6, 1.0)],
                parameter_count,
            )
            if use_doppler:
                los = normalize(sub(position, measurement.satellite_position_ecef_m))
                _add_row(
                    residuals,
                    jacobian,
                    doppler_residual(position, velocity, clock_drift, measurement),
                    measurement.pseudorange_rate_sigma_mps,
                    [(offset + 3, los[0]), (offset + 4, los[1]), (offset + 5, los[2]), (offset + 7, 1.0)],
                    parameter_count,
                )

    for epoch_index in range(1, len(dataset.epochs)):
        prev_position, prev_velocity, prev_clock_bias, prev_clock_drift = _read_state(parameters, epoch_index - 1)
        curr_position, curr_velocity, curr_clock_bias, curr_clock_drift = _read_state(parameters, epoch_index)
        dt = dataset.epochs[epoch_index].truth.time_seconds - dataset.epochs[epoch_index - 1].truth.time_seconds
        prev_offset = (epoch_index - 1) * 8
        curr_offset = epoch_index * 8
        for axis in range(3):
            _add_row(
                residuals,
                jacobian,
                curr_position[axis] - prev_position[axis] - prev_velocity[axis] * dt,
                3.0,
                [(curr_offset + axis, 1.0), (prev_offset + axis, -1.0), (prev_offset + 3 + axis, -dt)],
                parameter_count,
            )
            _add_row(
                residuals,
                jacobian,
                curr_velocity[axis] - prev_velocity[axis],
                1.0,
                [(curr_offset + 3 + axis, 1.0), (prev_offset + 3 + axis, -1.0)],
                parameter_count,
            )
        _add_row(
            residuals,
            jacobian,
            curr_clock_bias - prev_clock_bias - prev_clock_drift * dt,
            10.0,
            [(curr_offset + 6, 1.0), (prev_offset + 6, -1.0), (prev_offset + 7, -dt)],
            parameter_count,
        )
        _add_row(
            residuals,
            jacobian,
            curr_clock_drift - prev_clock_drift,
            1.0,
            [(curr_offset + 7, 1.0), (prev_offset + 7, -1.0)],
            parameter_count,
        )
    return jacobian, residuals


def run_fgo(dataset: SyntheticDataset) -> SolverRun:
    start = time.perf_counter()
    wls = run_wls(dataset)
    parameters = [0.0] * (len(dataset.epochs) * 8)
    for epoch_index, epoch in enumerate(dataset.epochs):
        wls_estimate = wls.estimates[epoch_index]
        _write_state(parameters, epoch_index, wls_estimate.position_ecef_m, epoch.truth.velocity_ecef_mps, wls_estimate.clock_bias_m, 0.0)
    history: list[dict[str, float | int | str]] = []
    converged = False
    final_residuals: list[float] = []
    for iteration in range(15):
        jacobian, residuals = _build_fgo_system(dataset, parameters, use_doppler=True)
        update = solve_normal_equations(jacobian, residuals, damping=1e-5)
        for i, value in enumerate(update):
            parameters[i] += value
        max_update = max(abs(value) for value in update)
        history.append({"estimator": "fgo", "iteration": iteration, "rms": rms(residuals), "max_update": max_update})
        final_residuals = residuals
        if max_update < 1e-4:
            converged = True
            break
    total_latency_ms = (time.perf_counter() - start) * 1000.0
    estimates = []
    for epoch_index, epoch in enumerate(dataset.epochs):
        position, velocity, clock_bias, clock_drift = _read_state(parameters, epoch_index)
        estimates.append(
            Estimate(
                "fgo",
                epoch_index,
                epoch.truth.time_seconds,
                position,
                velocity,
                clock_bias,
                clock_drift,
                converged,
                len(history),
                rms(final_residuals),
                total_latency_ms / max(len(dataset.epochs), 1),
                0 if converged else 1,
            )
        )
    return SolverRun(tuple(estimates), tuple(history))
