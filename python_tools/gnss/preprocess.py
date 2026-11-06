"""Convert replayed Android raw GNSS logs into clean observation epochs."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

SPEED_OF_LIGHT_MPS = 299_792_458.0
NANOS_PER_SECOND = 1_000_000_000
GPS_WEEK_NANOS = 604_800 * NANOS_PER_SECOND
GPS_DAY_NANOS = 86_400 * NANOS_PER_SECOND

CONSTELLATION_GPS = 1
CONSTELLATION_SBAS = 2
CONSTELLATION_GLONASS = 3
CONSTELLATION_QZSS = 4
CONSTELLATION_BEIDOU = 5
CONSTELLATION_GALILEO = 6
CONSTELLATION_IRNSS = 7

CONSTELLATION_NAMES = {
    CONSTELLATION_GPS: "GPS",
    CONSTELLATION_SBAS: "SBAS",
    CONSTELLATION_GLONASS: "GLONASS",
    CONSTELLATION_QZSS: "QZSS",
    CONSTELLATION_BEIDOU: "BEIDOU",
    CONSTELLATION_GALILEO: "GALILEO",
    CONSTELLATION_IRNSS: "IRNSS",
}

STATE_CODE_LOCK = 1 << 0
STATE_TOW_DECODED = 1 << 3
STATE_TOW_KNOWN = 1 << 14
STATE_GAL_E1BC_CODE_LOCK = 1 << 10
STATE_GAL_E1C_2ND_CODE_LOCK = 1 << 11
STATE_GLO_TOD_DECODED = 1 << 7
STATE_GLO_TOD_KNOWN = 1 << 15
STATE_BDS_D2_BIT_SYNC = 1 << 8
STATE_BDS_D2_SUBFRAME_SYNC = 1 << 9


@dataclass(frozen=True)
class GnssPreprocessConfig:
    min_cn0_dbhz: float = 18.0
    max_pseudorange_uncertainty_m: float = 300.0
    max_pseudorange_rate_uncertainty_mps: float = 20.0
    allowed_constellations: frozenset[int] = frozenset(
        {CONSTELLATION_GPS, CONSTELLATION_QZSS, CONSTELLATION_GALILEO, CONSTELLATION_BEIDOU}
    )
    allowed_frequencies_hz: tuple[tuple[float, float], ...] = (
        (1_559_000_000.0, 1_610_000_000.0),
        (1_164_000_000.0, 1_215_000_000.0),
    )
    min_pseudorange_m: float = 1.0e6
    max_pseudorange_m: float = 6.0e7


@dataclass(frozen=True)
class Observation:
    epoch_elapsed_realtime_nanos: int
    constellation_type: int
    constellation_name: str
    svid: int
    pseudorange_m: float
    pseudorange_uncertainty_m: float
    pseudorange_rate_mps: float
    pseudorange_rate_uncertainty_mps: float
    cn0_dbhz: float
    carrier_frequency_hz: float | None
    received_sv_time_nanos: int
    raw_state: int
    raw: dict[str, Any]


@dataclass(frozen=True)
class RejectedMeasurement:
    epoch_elapsed_realtime_nanos: int
    constellation_type: int | None
    svid: int | None
    reason: str
    detail: str = ""


@dataclass(frozen=True)
class ObservationEpoch:
    epoch_elapsed_realtime_nanos: int
    clock: dict[str, Any]
    observations: tuple[Observation, ...]
    rejected: tuple[RejectedMeasurement, ...] = field(default_factory=tuple)


def load_ndjson(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, 1):
            stripped = line.strip()
            if not stripped:
                continue
            record = json.loads(stripped)
            if not isinstance(record, dict):
                raise ValueError(f"line {line_no}: expected JSON object")
            records.append(record)
    return records


def preprocess_log(path: Path, config: GnssPreprocessConfig = GnssPreprocessConfig()) -> list[ObservationEpoch]:
    return preprocess_records(load_ndjson(path), config)


def preprocess_records(
    records: Iterable[dict[str, Any]],
    config: GnssPreprocessConfig = GnssPreprocessConfig(),
) -> list[ObservationEpoch]:
    current_clock: dict[str, Any] | None = None
    current_epoch_id: int | None = None
    current_measurements: list[dict[str, Any]] = []
    epochs: list[ObservationEpoch] = []

    def flush() -> None:
        nonlocal current_clock, current_epoch_id, current_measurements
        if current_clock is None or current_epoch_id is None:
            current_measurements = []
            return
        observations: list[Observation] = []
        rejected: list[RejectedMeasurement] = []
        for measurement in current_measurements:
            observation, rejection = build_observation(current_clock, measurement, config)
            if observation is not None:
                observations.append(observation)
            else:
                rejected.append(rejection)
        epochs.append(
            ObservationEpoch(
                epoch_elapsed_realtime_nanos=current_epoch_id,
                clock=current_clock,
                observations=tuple(observations),
                rejected=tuple(rejected),
            )
        )
        current_clock = None
        current_epoch_id = None
        current_measurements = []

    for record in records:
        record_type = record.get("record_type")
        if record_type == "gnss_clock":
            flush()
            current_clock = record
            current_epoch_id = int(record["elapsed_realtime_nanos"])
        elif record_type == "gnss_raw":
            epoch_id = int(record["gnss_event_elapsed_realtime_nanos"])
            if current_clock is None or current_epoch_id != epoch_id:
                flush()
                current_clock = None
                current_epoch_id = epoch_id
                current_measurements = []
                current_measurements.append(record)
            else:
                current_measurements.append(record)
    flush()
    return epochs


def build_observation(
    clock: dict[str, Any],
    measurement: dict[str, Any],
    config: GnssPreprocessConfig,
) -> tuple[Observation | None, RejectedMeasurement]:
    epoch_id = int(measurement.get("gnss_event_elapsed_realtime_nanos", measurement.get("elapsed_realtime_nanos", 0)))
    constellation = _optional_int(measurement.get("constellation_type"))
    svid = _optional_int(measurement.get("svid"))

    def reject(reason: str, detail: str = "") -> tuple[None, RejectedMeasurement]:
        return None, RejectedMeasurement(epoch_id, constellation, svid, reason, detail)

    if constellation is None or svid is None:
        return reject("missing_satellite_id")
    if constellation not in config.allowed_constellations:
        return reject("constellation_rejected", CONSTELLATION_NAMES.get(constellation, str(constellation)))

    cn0 = float(measurement["cn0_dbhz"])
    if not math.isfinite(cn0) or cn0 < config.min_cn0_dbhz:
        return reject("cn0_below_threshold", f"{cn0:.1f} < {config.min_cn0_dbhz:.1f}")

    frequency = measurement.get("carrier_frequency_hz")
    if frequency is not None:
        frequency = float(frequency)
        if not any(low <= frequency <= high for low, high in config.allowed_frequencies_hz):
            return reject("frequency_rejected", f"{frequency:.1f} Hz")

    state = int(measurement["state"])
    if not _has_valid_time_state(constellation, state):
        return reject("invalid_time_state", f"state={state}")

    if "full_bias_nanos" not in clock:
        return reject("missing_full_bias_nanos")

    received_sv_time_nanos = int(measurement["received_sv_time_nanos"])
    pseudorange_m = compute_pseudorange_m(clock, measurement)
    if pseudorange_m is None or not math.isfinite(pseudorange_m):
        return reject("pseudorange_unavailable")
    if pseudorange_m < config.min_pseudorange_m or pseudorange_m > config.max_pseudorange_m:
        return reject("pseudorange_out_of_range", f"{pseudorange_m:.3f} m")

    pseudorange_uncertainty_m = int(measurement["received_sv_time_uncertainty_nanos"]) * 1e-9 * SPEED_OF_LIGHT_MPS
    if pseudorange_uncertainty_m > config.max_pseudorange_uncertainty_m:
        return reject("pseudorange_uncertainty_too_large", f"{pseudorange_uncertainty_m:.3f} m")

    prr = float(measurement["pseudorange_rate_mps"])
    prr_unc = float(measurement["pseudorange_rate_uncertainty_mps"])
    if not math.isfinite(prr) or not math.isfinite(prr_unc):
        return reject("pseudorange_rate_invalid")
    if prr_unc > config.max_pseudorange_rate_uncertainty_mps:
        return reject("pseudorange_rate_uncertainty_too_large", f"{prr_unc:.3f} m/s")

    return (
        Observation(
            epoch_elapsed_realtime_nanos=epoch_id,
            constellation_type=constellation,
            constellation_name=CONSTELLATION_NAMES.get(constellation, str(constellation)),
            svid=svid,
            pseudorange_m=pseudorange_m,
            pseudorange_uncertainty_m=pseudorange_uncertainty_m,
            pseudorange_rate_mps=prr,
            pseudorange_rate_uncertainty_mps=prr_unc,
            cn0_dbhz=cn0,
            carrier_frequency_hz=frequency,
            received_sv_time_nanos=received_sv_time_nanos,
            raw_state=state,
            raw=measurement,
        ),
        RejectedMeasurement(epoch_id, constellation, svid, "accepted"),
    )


def compute_pseudorange_m(clock: dict[str, Any], measurement: dict[str, Any]) -> float | None:
    if "full_bias_nanos" not in clock:
        return None
    time_nanos = int(clock["time_nanos"])
    full_bias_nanos = int(clock["full_bias_nanos"])
    bias_nanos = float(clock.get("bias_nanos", 0.0))
    received_sv_time_nanos = int(measurement["received_sv_time_nanos"])
    constellation = int(measurement["constellation_type"])

    receiver_time_gnss_nanos = time_nanos - (full_bias_nanos + bias_nanos)
    if constellation == CONSTELLATION_GLONASS:
        receiver_time_mod_nanos = receiver_time_gnss_nanos % GPS_DAY_NANOS
    else:
        receiver_time_mod_nanos = receiver_time_gnss_nanos % GPS_WEEK_NANOS

    propagation_nanos = receiver_time_mod_nanos - received_sv_time_nanos
    if propagation_nanos < 0:
        propagation_nanos += GPS_DAY_NANOS if constellation == CONSTELLATION_GLONASS else GPS_WEEK_NANOS
    return propagation_nanos * 1e-9 * SPEED_OF_LIGHT_MPS


def _has_valid_time_state(constellation: int, state: int) -> bool:
    if not (state & STATE_CODE_LOCK):
        return False
    if constellation == CONSTELLATION_GLONASS:
        return bool(state & (STATE_GLO_TOD_DECODED | STATE_GLO_TOD_KNOWN))
    if constellation == CONSTELLATION_BEIDOU:
        return bool(state & (STATE_TOW_DECODED | STATE_TOW_KNOWN | STATE_BDS_D2_SUBFRAME_SYNC))
    if constellation == CONSTELLATION_GALILEO:
        return bool(state & (STATE_TOW_DECODED | STATE_TOW_KNOWN | STATE_GAL_E1BC_CODE_LOCK | STATE_GAL_E1C_2ND_CODE_LOCK))
    return bool(state & (STATE_TOW_DECODED | STATE_TOW_KNOWN))


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    return int(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--min-cn0", type=float, default=GnssPreprocessConfig.min_cn0_dbhz)
    args = parser.parse_args()

    epochs = preprocess_log(args.log, GnssPreprocessConfig(min_cn0_dbhz=args.min_cn0))
    accepted = sum(len(epoch.observations) for epoch in epochs)
    rejected = sum(len(epoch.rejected) for epoch in epochs)
    print(f"epochs: {len(epochs)}")
    print(f"accepted_observations: {accepted}")
    print(f"rejected_measurements: {rejected}")
    for epoch in epochs[:5]:
        print(
            f"epoch {epoch.epoch_elapsed_realtime_nanos}: "
            f"accepted={len(epoch.observations)} rejected={len(epoch.rejected)}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
