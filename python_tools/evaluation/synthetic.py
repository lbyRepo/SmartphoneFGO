"""Synthetic GNSS dataset matching the C++ estimator-core fixture."""

from __future__ import annotations

from dataclasses import dataclass

from python_tools.evaluation.math_utils import Vec3, dot, norm, scale, sub


@dataclass(frozen=True)
class SyntheticSatellite:
    svid: int
    position_ecef_m: Vec3
    velocity_ecef_mps: Vec3


@dataclass(frozen=True)
class SyntheticMeasurement:
    svid: int
    satellite_position_ecef_m: Vec3
    satellite_velocity_ecef_mps: Vec3
    pseudorange_m: float
    pseudorange_sigma_m: float
    pseudorange_rate_mps: float
    pseudorange_rate_sigma_mps: float


@dataclass(frozen=True)
class ReceiverTruth:
    time_seconds: float
    position_ecef_m: Vec3
    velocity_ecef_mps: Vec3
    clock_bias_m: float
    clock_drift_mps: float


@dataclass(frozen=True)
class SyntheticEpoch:
    truth: ReceiverTruth
    measurements: tuple[SyntheticMeasurement, ...]


@dataclass(frozen=True)
class SyntheticDataset:
    epochs: tuple[SyntheticEpoch, ...]
    fused_positions_ecef_m: tuple[Vec3, ...]


SATELLITES = (
    SyntheticSatellite(1, (15600000.0, 21700000.0, 20100000.0), (-820.0, 510.0, 40.0)),
    SyntheticSatellite(2, (-18700000.0, 13400000.0, 21400000.0), (420.0, -760.0, 110.0)),
    SyntheticSatellite(3, (21100000.0, -14300000.0, 17800000.0), (120.0, 690.0, -150.0)),
    SyntheticSatellite(4, (17600000.0, 18600000.0, -19100000.0), (-500.0, -210.0, 620.0)),
    SyntheticSatellite(5, (-15200000.0, -21300000.0, 19800000.0), (730.0, 130.0, -90.0)),
    SyntheticSatellite(6, (23200000.0, 7300000.0, 20400000.0), (-240.0, 430.0, -550.0)),
    SyntheticSatellite(7, (-9300000.0, 23800000.0, -18400000.0), (360.0, -290.0, 480.0)),
    SyntheticSatellite(8, (12500000.0, -22800000.0, -19600000.0), (-180.0, 350.0, 530.0)),
)


def make_measurement(
    satellite: SyntheticSatellite,
    receiver: ReceiverTruth,
    pseudorange_noise_m: float,
    range_rate_noise_mps: float,
) -> SyntheticMeasurement:
    sat_to_receiver = sub(receiver.position_ecef_m, satellite.position_ecef_m)
    geometric_range = norm(sat_to_receiver)
    los_sat_to_rx = scale(sat_to_receiver, 1.0 / geometric_range)
    range_rate = dot(los_sat_to_rx, sub(receiver.velocity_ecef_mps, satellite.velocity_ecef_mps))
    return SyntheticMeasurement(
        svid=satellite.svid,
        satellite_position_ecef_m=satellite.position_ecef_m,
        satellite_velocity_ecef_mps=satellite.velocity_ecef_mps,
        pseudorange_m=geometric_range + receiver.clock_bias_m + pseudorange_noise_m,
        pseudorange_sigma_m=1.0,
        pseudorange_rate_mps=range_rate + receiver.clock_drift_mps + range_rate_noise_mps,
        pseudorange_rate_sigma_mps=0.2,
    )


def make_synthetic_dataset() -> SyntheticDataset:
    epochs: list[SyntheticEpoch] = []
    fused: list[Vec3] = []
    for epoch_index in range(4):
        t = float(epoch_index)
        truth = ReceiverTruth(
            time_seconds=t,
            position_ecef_m=(-2700400.0 + 12.0 * t, 4285150.0 - 4.0 * t, 3850120.0 + 2.5 * t),
            velocity_ecef_mps=(12.0, -4.0, 2.5),
            clock_bias_m=48.0 + 0.15 * t,
            clock_drift_mps=0.15,
        )
        measurements = []
        for i, satellite in enumerate(SATELLITES):
            pr_noise = (i % 3 - 1) * 0.35 + 0.05 * epoch_index
            rr_noise = (i % 4 - 1.5) * 0.015
            measurements.append(make_measurement(satellite, truth, pr_noise, rr_noise))
        epochs.append(SyntheticEpoch(truth, tuple(measurements)))
        fused.append(
            (
                truth.position_ecef_m[0] + 3.0 - 0.2 * epoch_index,
                truth.position_ecef_m[1] - 2.0 + 0.15 * epoch_index,
                truth.position_ecef_m[2] + 1.0,
            )
        )
    return SyntheticDataset(tuple(epochs), tuple(fused))
