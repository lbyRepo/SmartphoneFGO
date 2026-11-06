#!/usr/bin/env python3
"""Run a deterministic WLS and fixed-lag GNSS FGO benchmark on GSDC data.

The input is the Google-derived CSV format used by the Smartphone Decimeter
Challenge. This module deliberately consumes satellite positions and corrections
already present in that format; it does not pretend to be an ephemeris decoder.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


C_MPS = 299_792_458.0
OMEGA_E_RADPS = 7.2921151467e-5
WGS84_A_M = 6_378_137.0
WGS84_F = 1.0 / 298.257223563
SUPPORTED_CONSTELLATIONS = {1, 3, 4, 5, 6}  # GPS, GLONASS, QZSS, BeiDou, Galileo
RICH_DERIVED_SCHEMA = "gsdc_2023_rich_derived"
LEGACY_DERIVED_SCHEMA = "gsdc_2021_legacy_derived"


@dataclass(frozen=True)
class Observation:
    timestamp_ms: int
    constellation: int
    svid: int
    signal_type: str
    satellite_position_m: np.ndarray
    satellite_velocity_mps: np.ndarray
    corrected_pseudorange_m: float
    pseudorange_sigma_m: float
    corrected_range_rate_mps: float
    range_rate_sigma_mps: float
    cn0_dbhz: float
    elevation_deg: float


@dataclass(frozen=True)
class Epoch:
    timestamp_ms: int
    observations: tuple[Observation, ...]


@dataclass(frozen=True)
class Estimate:
    timestamp_ms: int
    position_m: np.ndarray
    velocity_mps: np.ndarray
    clock_bias_m: float
    clock_drift_mps: float
    iterations: int
    observation_count: int
    rejected_count: int
    latency_ms: float
    converged: bool


def _float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    value = row.get(key, "")
    if value is None or value.strip() == "":
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def detect_derived_schema(fieldnames: Sequence[str] | None) -> str:
    fields = set(fieldnames or ())
    if {"MessageType", "utcTimeMillis", "RawPseudorangeMeters"} <= fields:
        return RICH_DERIVED_SCHEMA
    if {"millisSinceGpsEpoch", "rawPrM", "xSatPosM"} <= fields:
        return LEGACY_DERIVED_SCHEMA
    raise ValueError(f"Unsupported derived GNSS CSV columns: {sorted(fields)}")


def load_derived_epochs(path: Path) -> tuple[list[Epoch], dict[str, int]]:
    grouped: dict[int, list[Observation]] = {}
    rejection_counts: dict[str, int] = {}

    def reject(reason: str) -> None:
        rejection_counts[reason] = rejection_counts.get(reason, 0) + 1

    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        schema = detect_derived_schema(reader.fieldnames)
        for row in reader:
            if schema == RICH_DERIVED_SCHEMA and row.get("MessageType") != "Raw":
                reject("not_raw")
                continue
            if schema == RICH_DERIVED_SCHEMA:
                timestamp = int(_float(row, "utcTimeMillis", -1.0))
                constellation = int(_float(row, "ConstellationType", -1.0))
                svid = int(_float(row, "Svid", -1.0))
                state = int(_float(row, "State", 0.0))
                carrier_frequency = _float(row, "CarrierFrequencyHz")
                cn0 = _float(row, "Cn0DbHz")
                elevation = _float(row, "SvElevationDegrees")
                raw_pr = _float(row, "RawPseudorangeMeters")
                pr_sigma = _float(row, "RawPseudorangeUncertaintyMeters")
                sat_clock = _float(row, "SvClockBiasMeters")
                isrb = _float(row, "IsrbMeters", 0.0)
                iono = _float(row, "IonosphericDelayMeters", 0.0)
                tropo = _float(row, "TroposphericDelayMeters", 0.0)
                raw_rate = _float(row, "PseudorangeRateMetersPerSecond")
                rate_sigma = _float(row, "PseudorangeRateUncertaintyMetersPerSecond")
                sat_clock_drift = _float(row, "SvClockDriftMetersPerSecond", 0.0)
                signal_type = row.get("SignalType", "UNKNOWN")
                sat_pos = np.array(
                    [_float(row, "SvPositionXEcefMeters"),
                     _float(row, "SvPositionYEcefMeters"),
                     _float(row, "SvPositionZEcefMeters")],
                    dtype=float,
                )
                sat_vel = np.array(
                    [_float(row, "SvVelocityXEcefMetersPerSecond"),
                     _float(row, "SvVelocityYEcefMetersPerSecond"),
                     _float(row, "SvVelocityZEcefMetersPerSecond")],
                    dtype=float,
                )
            else:
                timestamp = int(_float(row, "millisSinceGpsEpoch", -1.0))
                constellation = int(_float(row, "constellationType", -1.0))
                svid = int(_float(row, "svid", -1.0))
                state = 1
                carrier_frequency = math.nan
                cn0 = math.nan
                elevation = math.nan
                raw_pr = _float(row, "rawPrM")
                pr_sigma = _float(row, "rawPrUncM")
                sat_clock = _float(row, "satClkBiasM")
                isrb = _float(row, "isrbM", 0.0)
                iono = _float(row, "ionoDelayM", 0.0)
                tropo = _float(row, "tropoDelayM", 0.0)
                raw_rate = math.nan
                rate_sigma = math.nan
                sat_clock_drift = _float(row, "satClkDriftMps", 0.0)
                signal_type = row.get("signalType", "UNKNOWN")
                sat_pos = np.array(
                    [_float(row, "xSatPosM"), _float(row, "ySatPosM"),
                     _float(row, "zSatPosM")],
                    dtype=float,
                )
                sat_vel = np.array(
                    [_float(row, "xSatVelMps"), _float(row, "ySatVelMps"),
                     _float(row, "zSatVelMps")],
                    dtype=float,
                )

            if timestamp < 0 or constellation not in SUPPORTED_CONSTELLATIONS:
                reject("unsupported_constellation_or_time")
                continue
            if state == 0:
                reject("invalid_tracking_state")
                continue
            if schema == RICH_DERIVED_SCHEMA:
                if not math.isfinite(carrier_frequency) or not 1.1e9 <= carrier_frequency <= 1.7e9:
                    reject("unsupported_frequency")
                    continue
                if not _finite([cn0, elevation]) or cn0 < 18.0 or elevation < 5.0:
                    reject("low_signal_or_elevation")
                    continue
            if not _finite([raw_pr, pr_sigma, sat_clock, isrb, iono, tropo]):
                reject("missing_pseudorange_input")
                continue
            if raw_pr < 1.0e6 or raw_pr > 1.0e8 or pr_sigma <= 0.0 or pr_sigma > 100.0:
                reject("invalid_pseudorange")
                continue
            if not np.isfinite(sat_pos).all() or np.linalg.norm(sat_pos) < 2.0e7:
                reject("invalid_satellite_position")
                continue

            corrected_pr = raw_pr + sat_clock - isrb - iono - tropo
            if _finite([raw_rate, rate_sigma, sat_clock_drift]) and 0.0 < rate_sigma <= 20.0:
                corrected_rate = raw_rate + sat_clock_drift
            else:
                corrected_rate = math.nan
                rate_sigma = math.nan
                sat_vel[:] = math.nan

            observation = Observation(
                timestamp_ms=timestamp,
                constellation=constellation,
                svid=svid,
                signal_type=signal_type,
                satellite_position_m=sat_pos,
                satellite_velocity_mps=sat_vel,
                corrected_pseudorange_m=corrected_pr,
                pseudorange_sigma_m=max(pr_sigma, 1.0),
                corrected_range_rate_mps=corrected_rate,
                range_rate_sigma_mps=max(rate_sigma, 0.15) if math.isfinite(rate_sigma) else math.nan,
                cn0_dbhz=cn0,
                elevation_deg=elevation,
            )
            grouped.setdefault(timestamp, []).append(observation)

    epochs = [Epoch(timestamp, tuple(grouped[timestamp])) for timestamp in sorted(grouped)]
    epochs = [epoch for epoch in epochs if len(epoch.observations) >= 4]
    return epochs, rejection_counts


def load_ground_truth(path: Path) -> dict[int, np.ndarray]:
    truth: dict[int, np.ndarray] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        rich_schema = {"MessageType", "UnixTimeMillis", "LatitudeDegrees"} <= fields
        legacy_schema = {"millisSinceGpsEpoch", "latDeg", "lngDeg"} <= fields
        if not rich_schema and not legacy_schema:
            raise ValueError(f"Unsupported ground-truth CSV columns: {sorted(fields)}")
        for row in reader:
            if rich_schema:
                if row.get("MessageType") != "Fix":
                    continue
                timestamp = int(_float(row, "UnixTimeMillis", -1.0))
                lla = [_float(row, "LatitudeDegrees"), _float(row, "LongitudeDegrees"),
                       _float(row, "AltitudeMeters")]
            else:
                timestamp = int(_float(row, "millisSinceGpsEpoch", -1.0))
                lla = [_float(row, "latDeg"), _float(row, "lngDeg"),
                       _float(row, "heightAboveWgs84EllipsoidM")]
            if timestamp >= 0 and _finite(lla):
                truth[timestamp] = lla_to_ecef(*lla)
    return truth


def lla_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> np.ndarray:
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    e2 = WGS84_F * (2.0 - WGS84_F)
    n = WGS84_A_M / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    return np.array([
        (n + altitude_m) * math.cos(lat) * math.cos(lon),
        (n + altitude_m) * math.cos(lat) * math.sin(lon),
        (n * (1.0 - e2) + altitude_m) * math.sin(lat),
    ])


def ecef_to_lla(position_m: Sequence[float]) -> np.ndarray:
    x, y, z = map(float, position_m)
    e2 = WGS84_F * (2.0 - WGS84_F)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    altitude = 0.0
    for _ in range(12):
        n = WGS84_A_M / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
        altitude = p / max(math.cos(lat), 1.0e-12) - n
        updated = math.atan2(z, p * (1.0 - e2 * n / (n + altitude)))
        if abs(updated - lat) < 1.0e-13:
            lat = updated
            break
        lat = updated
    return np.array([math.degrees(lat), math.degrees(lon), altitude])


def ecef_delta_to_enu(delta_m: np.ndarray, reference_ecef_m: np.ndarray) -> np.ndarray:
    lat_deg, lon_deg, _ = ecef_to_lla(reference_ecef_m)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    rotation = np.array([
        [-math.sin(lon), math.cos(lon), 0.0],
        [-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat)],
        [math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)],
    ])
    return rotation @ delta_m


def _rotate_satellite(satellite_m: np.ndarray, transit_seconds: float) -> np.ndarray:
    theta = OMEGA_E_RADPS * transit_seconds
    cosine, sine = math.cos(theta), math.sin(theta)
    x, y, z = satellite_m
    return np.array([cosine * x + sine * y, -sine * x + cosine * y, z])


def _huber_weight(standardized_residual: float, cutoff: float = 2.5) -> float:
    magnitude = abs(standardized_residual)
    return 1.0 if magnitude <= cutoff else cutoff / magnitude


def run_wls(epochs: Sequence[Epoch]) -> list[Estimate]:
    estimates: list[Estimate] = []
    state = np.zeros(4)
    for epoch in epochs:
        start = time.perf_counter_ns()
        converged = False
        rejected = 0
        iterations = 0
        for iterations in range(1, 21):
            rows: list[np.ndarray] = []
            residuals: list[float] = []
            for obs in epoch.observations:
                transit = (obs.corrected_pseudorange_m - state[3]) / C_MPS
                sat = _rotate_satellite(obs.satellite_position_m, transit)
                delta = state[:3] - sat
                distance = np.linalg.norm(delta)
                if distance < 1.0:
                    continue
                unit = delta / distance
                residual = distance + state[3] - obs.corrected_pseudorange_m
                standardized = residual / obs.pseudorange_sigma_m
                robust = _huber_weight(standardized)
                scale = math.sqrt(robust) / obs.pseudorange_sigma_m
                rows.append(np.array([unit[0], unit[1], unit[2], 1.0]) * scale)
                residuals.append(residual * scale)
            if len(rows) < 4:
                break
            jacobian = np.vstack(rows)
            residual_vector = np.asarray(residuals)
            step, *_ = np.linalg.lstsq(jacobian, -residual_vector, rcond=1.0e-12)
            state += step
            if np.linalg.norm(step[:3]) < 1.0e-3 and abs(step[3]) < 1.0e-3:
                converged = True
                break
        for obs in epoch.observations:
            sat = _rotate_satellite(
                obs.satellite_position_m,
                (obs.corrected_pseudorange_m - state[3]) / C_MPS,
            )
            residual = np.linalg.norm(state[:3] - sat) + state[3] - obs.corrected_pseudorange_m
            rejected += int(abs(residual / obs.pseudorange_sigma_m) > 8.0)
        elapsed = (time.perf_counter_ns() - start) / 1.0e6
        estimates.append(Estimate(
            epoch.timestamp_ms, state[:3].copy(), np.zeros(3), float(state[3]), 0.0,
            iterations, len(epoch.observations), rejected, elapsed, converged,
        ))
    return estimates


def _pseudorange_factor(state: np.ndarray, obs: Observation) -> tuple[float, np.ndarray]:
    transit = (obs.corrected_pseudorange_m - state[6]) / C_MPS
    sat = _rotate_satellite(obs.satellite_position_m, transit)
    delta = state[:3] - sat
    distance = np.linalg.norm(delta)
    unit = delta / distance
    residual = distance + state[6] - obs.corrected_pseudorange_m
    jacobian = np.zeros(8)
    jacobian[:3] = unit
    jacobian[6] = 1.0
    return residual, jacobian


def _doppler_factor(state: np.ndarray, obs: Observation) -> tuple[float, np.ndarray]:
    transit = (obs.corrected_pseudorange_m - state[6]) / C_MPS
    sat = _rotate_satellite(obs.satellite_position_m, transit)
    delta = state[:3] - sat
    distance = np.linalg.norm(delta)
    unit = delta / distance
    relative_velocity = state[3:6] - obs.satellite_velocity_mps
    predicted = float(unit @ relative_velocity + state[7])
    jacobian = np.zeros(8)
    projection = relative_velocity - unit * float(unit @ relative_velocity)
    jacobian[:3] = projection / distance
    jacobian[3:6] = unit
    jacobian[7] = 1.0
    return predicted - obs.corrected_range_rate_mps, jacobian


def check_factor_jacobians(state: np.ndarray, obs: Observation) -> dict[str, float]:
    results: dict[str, float] = {}
    factors = [("pseudorange", _pseudorange_factor)]
    if (math.isfinite(obs.corrected_range_rate_mps) and
            math.isfinite(obs.range_rate_sigma_mps) and
            np.isfinite(obs.satellite_velocity_mps).all()):
        factors.append(("doppler", _doppler_factor))
    for name, factor in factors:
        _, analytic = factor(state, obs)
        numerical = np.zeros(8)
        for index in range(8):
            epsilon = 1.0e-3 if index < 7 else 1.0e-4
            plus, minus = state.copy(), state.copy()
            plus[index] += epsilon
            minus[index] -= epsilon
            numerical[index] = (factor(plus, obs)[0] - factor(minus, obs)[0]) / (2.0 * epsilon)
        results[name] = float(np.max(np.abs(analytic - numerical)))
    return results


def _solve_window(epochs: Sequence[Epoch], wls: Sequence[Estimate]) -> tuple[np.ndarray, int, bool]:
    count = len(epochs)
    states = np.zeros((count, 8))
    for index, estimate in enumerate(wls):
        states[index, :3] = estimate.position_m
        states[index, 6] = estimate.clock_bias_m
    for index in range(1, count):
        dt = max((epochs[index].timestamp_ms - epochs[index - 1].timestamp_ms) / 1000.0, 1.0e-3)
        states[index - 1, 3:6] = (states[index, :3] - states[index - 1, :3]) / dt
        states[index - 1, 7] = (states[index, 6] - states[index - 1, 6]) / dt
    if count > 1:
        states[-1, 3:6] = states[-2, 3:6]
        states[-1, 7] = states[-2, 7]

    converged = False
    iterations = 0
    for iterations in range(1, 16):
        rows: list[np.ndarray] = []
        residuals: list[float] = []

        def append_factor(epoch_index: int, residual: float, local_jac: np.ndarray,
                          sigma: float, robust: bool = False) -> None:
            standardized = residual / sigma
            weight = _huber_weight(standardized) if robust else 1.0
            scale = math.sqrt(weight) / sigma
            row = np.zeros(count * 8)
            row[epoch_index * 8:(epoch_index + 1) * 8] = local_jac * scale
            rows.append(row)
            residuals.append(residual * scale)

        prior_sigmas = np.array([30.0, 30.0, 30.0, 20.0, 20.0, 20.0, 1000.0, 100.0])
        prior = np.zeros(8)
        prior[:3] = wls[0].position_m
        prior[6] = wls[0].clock_bias_m
        for component in range(8):
            jac = np.zeros(8)
            jac[component] = 1.0
            append_factor(0, states[0, component] - prior[component], jac,
                          prior_sigmas[component])

        for epoch_index, epoch in enumerate(epochs):
            for obs in epoch.observations:
                residual, jac = _pseudorange_factor(states[epoch_index], obs)
                append_factor(epoch_index, residual, jac, obs.pseudorange_sigma_m, robust=True)
                if math.isfinite(obs.corrected_range_rate_mps) and np.isfinite(obs.satellite_velocity_mps).all():
                    residual, jac = _doppler_factor(states[epoch_index], obs)
                    append_factor(epoch_index, residual, jac, obs.range_rate_sigma_mps, robust=True)

        for index in range(1, count):
            dt = max((epochs[index].timestamp_ms - epochs[index - 1].timestamp_ms) / 1000.0, 1.0e-3)
            offset_previous = (index - 1) * 8
            offset_current = index * 8
            for axis in range(3):
                row = np.zeros(count * 8)
                row[offset_current + axis] = 1.0 / 3.0
                row[offset_previous + axis] = -1.0 / 3.0
                row[offset_previous + 3 + axis] = -dt / 3.0
                residual = states[index, axis] - states[index - 1, axis] - dt * states[index - 1, 3 + axis]
                rows.append(row)
                residuals.append(residual / 3.0)
                row = np.zeros(count * 8)
                row[offset_current + 3 + axis] = 1.0 / 3.0
                row[offset_previous + 3 + axis] = -1.0 / 3.0
                rows.append(row)
                residuals.append((states[index, 3 + axis] - states[index - 1, 3 + axis]) / 3.0)
            row = np.zeros(count * 8)
            row[offset_current + 6] = 1.0 / 100.0
            row[offset_previous + 6] = -1.0 / 100.0
            row[offset_previous + 7] = -dt / 100.0
            rows.append(row)
            residuals.append((states[index, 6] - states[index - 1, 6] - dt * states[index - 1, 7]) / 100.0)
            row = np.zeros(count * 8)
            row[offset_current + 7] = 1.0 / 50.0
            row[offset_previous + 7] = -1.0 / 50.0
            rows.append(row)
            residuals.append((states[index, 7] - states[index - 1, 7]) / 50.0)

        jacobian = np.vstack(rows)
        residual_vector = np.asarray(residuals)
        damping = 1.0e-6
        augmented_jacobian = np.vstack([jacobian, math.sqrt(damping) * np.eye(count * 8)])
        augmented_residual = np.concatenate([residual_vector, np.zeros(count * 8)])
        step, *_ = np.linalg.lstsq(augmented_jacobian, -augmented_residual, rcond=1.0e-12)
        states += step.reshape(count, 8)
        if np.max(np.linalg.norm(step.reshape(count, 8)[:, :3], axis=1)) < 1.0e-3:
            converged = True
            break
    return states, iterations, converged


def run_fixed_lag_fgo(epochs: Sequence[Epoch], wls: Sequence[Estimate], lag_seconds: float) -> list[Estimate]:
    estimates: list[Estimate] = []
    for end in range(len(epochs)):
        start_index = end
        while (start_index > 0 and
               epochs[end].timestamp_ms - epochs[start_index - 1].timestamp_ms <= lag_seconds * 1000.0):
            start_index -= 1
        start = time.perf_counter_ns()
        states, iterations, converged = _solve_window(
            epochs[start_index:end + 1], wls[start_index:end + 1]
        )
        state = states[-1]
        rejected = 0
        for obs in epochs[end].observations:
            residual, _ = _pseudorange_factor(state, obs)
            rejected += int(abs(residual / obs.pseudorange_sigma_m) > 8.0)
        elapsed = (time.perf_counter_ns() - start) / 1.0e6
        estimates.append(Estimate(
            epochs[end].timestamp_ms, state[:3].copy(), state[3:6].copy(),
            float(state[6]), float(state[7]), iterations, len(epochs[end].observations),
            rejected, elapsed, converged,
        ))
    return estimates


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    return float(np.percentile(values, percentile)) if values else None


def compute_metrics(estimates: Sequence[Estimate], truth: dict[int, np.ndarray]) -> dict[str, object]:
    horizontal: list[float] = []
    vertical: list[float] = []
    matched_positions: list[np.ndarray] = []
    for estimate in estimates:
        reference = truth.get(estimate.timestamp_ms)
        if reference is None:
            reference = truth.get(estimate.timestamp_ms - 1)
        if reference is None:
            reference = truth.get(estimate.timestamp_ms + 1)
        if reference is None:
            continue
        enu = ecef_delta_to_enu(estimate.position_m - reference, reference)
        horizontal.append(float(math.hypot(enu[0], enu[1])))
        vertical.append(float(abs(enu[2])))
        matched_positions.append(estimate.position_m)
    jumps = [float(np.linalg.norm(matched_positions[i] - matched_positions[i - 1]))
             for i in range(1, len(matched_positions))]
    accelerations = [float(np.linalg.norm(matched_positions[i] - 2.0 * matched_positions[i - 1]
                                         + matched_positions[i - 2]))
                     for i in range(2, len(matched_positions))]
    latency = [estimate.latency_ms for estimate in estimates]
    return {
        "matched_epochs": len(horizontal),
        "horizontal_error_m": {
            "mean": float(np.mean(horizontal)) if horizontal else None,
            "p50": _percentile(horizontal, 50.0),
            "p95": _percentile(horizontal, 95.0),
            "max": max(horizontal) if horizontal else None,
        },
        "vertical_error_m": {
            "mean": float(np.mean(vertical)) if vertical else None,
            "p95": _percentile(vertical, 95.0),
        },
        "max_position_jump_m": max(jumps) if jumps else 0.0,
        "trajectory_second_difference_rms_m": (
            float(math.sqrt(np.mean(np.square(accelerations)))) if accelerations else 0.0
        ),
        "latency_ms": {"mean": float(np.mean(latency)), "max": max(latency)},
        "solver_failures": sum(not estimate.converged for estimate in estimates),
        "rejected_measurements": sum(estimate.rejected_count for estimate in estimates),
        "mean_observations_per_epoch": float(np.mean([e.observation_count for e in estimates])),
    }


def write_outputs(output_dir: Path, epochs: Sequence[Epoch], truth: dict[int, np.ndarray],
                  wls: Sequence[Estimate], fgo: Sequence[Estimate], summary: dict[str, object]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "solutions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp_ms", "estimator", "x_ecef_m", "y_ecef_m", "z_ecef_m",
                         "vx_ecef_mps", "vy_ecef_mps", "vz_ecef_mps", "clock_bias_m",
                         "clock_drift_mps", "iterations", "latency_ms", "converged"])
        for name, estimates in (("wls", wls), ("fgo", fgo)):
            for estimate in estimates:
                writer.writerow([estimate.timestamp_ms, name, *estimate.position_m,
                                 *estimate.velocity_mps, estimate.clock_bias_m,
                                 estimate.clock_drift_mps, estimate.iterations,
                                 estimate.latency_ms, int(estimate.converged)])
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    _write_svg(output_dir / "trajectory.svg", truth, wls, fgo)


def _write_svg(path: Path, truth: dict[int, np.ndarray], wls: Sequence[Estimate],
               fgo: Sequence[Estimate]) -> None:
    if not truth:
        path.write_text("<svg xmlns=\"http://www.w3.org/2000/svg\"/>\n", encoding="utf-8")
        return
    reference = next(iter(truth.values()))
    series: list[tuple[str, str, list[np.ndarray]]] = []
    truth_enu = [ecef_delta_to_enu(value - reference, reference) for value in truth.values()]
    series.append(("Ground truth", "#151515", truth_enu))
    for name, color, estimates in (("WLS", "#d1495b", wls), ("FGO", "#00798c", fgo)):
        points = [ecef_delta_to_enu(estimate.position_m - reference, reference) for estimate in estimates]
        series.append((name, color, points))
    all_points = [point for _, _, points in series for point in points]
    east = [point[0] for point in all_points]
    north = [point[1] for point in all_points]
    min_e, max_e = min(east), max(east)
    min_n, max_n = min(north), max(north)
    span = max(max_e - min_e, max_n - min_n, 1.0)

    def screen(point: np.ndarray) -> tuple[float, float]:
        return 70.0 + 600.0 * (point[0] - min_e) / span, 650.0 - 600.0 * (point[1] - min_n) / span

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="760" height="720" viewBox="0 0 760 720">',
        '<rect width="760" height="720" fill="#f7f8fa"/>',
        '<text x="40" y="36" font-family="sans-serif" font-size="20" fill="#151515">GSDC local ENU trajectory</text>',
        '<rect x="70" y="50" width="600" height="600" fill="#ffffff" stroke="#ccd1d7"/>',
    ]
    for index, (name, color, points) in enumerate(series):
        coords = " ".join(f"{x:.2f},{y:.2f}" for x, y in map(screen, points))
        lines.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="3"/>')
        lines.append(f'<circle cx="{700}" cy="{80 + index * 28}" r="6" fill="{color}"/>')
        lines.append(f'<text x="714" y="{86 + index * 28}" font-family="sans-serif" font-size="13">{name}</text>')
    lines.append('<text x="315" y="695" font-family="sans-serif" font-size="13">East (scaled equally with North)</text>')
    lines.append('</svg>')
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--lag-seconds", type=float, default=2.0)
    parser.add_argument("--dataset-name")
    parser.add_argument("--dataset-scope")
    args = parser.parse_args()

    epochs, preprocessing_rejections = load_derived_epochs(args.device_gnss)
    truth = load_ground_truth(args.ground_truth)
    if not epochs:
        raise SystemExit("No usable GNSS epochs after preprocessing")
    wls = run_wls(epochs)
    fgo = run_fixed_lag_fgo(epochs, wls, args.lag_seconds)
    jacobian_state = np.zeros(8)
    jacobian_state[:3] = wls[0].position_m
    jacobian_state[6] = wls[0].clock_bias_m
    jacobians = check_factor_jacobians(jacobian_state, epochs[0].observations[0])
    if any(error > 2.0e-5 for error in jacobians.values()):
        raise SystemExit(f"Numerical Jacobian check failed: {jacobians}")
    with args.device_gnss.open(newline="", encoding="utf-8") as stream:
        schema = detect_derived_schema(csv.DictReader(stream).fieldnames)
    summary: dict[str, object] = {
        "dataset": args.dataset_name or args.device_gnss.stem,
        "dataset_scope": args.dataset_scope or "Public derived GNSS replay",
        "input_schema": schema,
        "lag_seconds": args.lag_seconds,
        "input_epochs": len(epochs),
        "preprocessing_rejections": preprocessing_rejections,
        "jacobian_max_abs_error": jacobians,
        "wls": compute_metrics(wls, truth),
        "fgo": compute_metrics(fgo, truth),
    }
    write_outputs(args.output, epochs, truth, wls, fgo, summary)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
