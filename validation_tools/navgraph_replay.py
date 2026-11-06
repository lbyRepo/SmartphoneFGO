#!/usr/bin/env python3
"""Deterministically replay NavGraph NDJSON logs through GNSS preprocessing.

The raw Android log remains the source of truth. Navigation messages and fused
fixes are consumed causally on the elapsed-realtime time base; GNSS receive time
is reconstructed only from GnssClock fields. The resulting clean observations
use the same correction signs as the Android preprocessing layer and can be fed
to the shared WLS/FGO benchmark implementation.
"""

from __future__ import annotations

import argparse
import base64
import csv
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from gsdc_benchmark import (
    C_MPS,
    Epoch,
    Estimate,
    Observation,
    check_factor_jacobians,
    ecef_delta_to_enu,
    lla_to_ecef,
    run_fixed_lag_fgo,
    run_wls,
)
from validate_ndjson import validate


GPS_WEEK_NS = 604_800_000_000_000
HALF_GPS_WEEK_NS = GPS_WEEK_NS // 2
GPS_L1_HZ = 1_575_420_000.0
GPS_L1_TOLERANCE_HZ = 5_000_000.0
GPS_L1_CA_TYPE = 257
GPS_CONSTELLATION = 1
MIN_CN0_DBHZ = 18.0
MIN_ELEVATION_DEG = 5.0
MAX_SV_TIME_UNCERTAINTY_NS = 334
MAX_EPHEMERIS_AGE_S = 14_400.0
MAX_SUBFRAME_SPAN_NS = 120_000_000_000
OMEGA_E_RADPS = 7.2921151467e-5
MU_M3PS2 = 3.986005e14
RELATIVITY_S_PER_SQRT_M = -4.442807633e-10


@dataclass(frozen=True)
class BroadcastEphemeris:
    svid: int
    week_modulo_1024: int
    ura_index: int
    sv_health: int
    iodc: int
    iode: int
    tgd_s: float
    toc_s: float
    af0_s: float
    af1_sps: float
    af2_sps2: float
    crs_m: float
    delta_n_radps: float
    m0_rad: float
    cuc_rad: float
    eccentricity: float
    cus_rad: float
    sqrt_a_sqrt_m: float
    toe_s: float
    cic_rad: float
    omega0_rad: float
    cis_rad: float
    i0_rad: float
    crc_m: float
    omega_rad: float
    omega_dot_radps: float
    i_dot_radps: float


@dataclass(frozen=True)
class SatelliteState:
    position_m: np.ndarray
    velocity_mps: np.ndarray
    clock_bias_m: float
    clock_drift_mps: float
    source_age_s: float
    accuracy_m: float


@dataclass
class NavigationHealth:
    received_messages: int = 0
    accepted_subframes: int = 0
    rejected_messages: int = 0
    complete_ephemerides: int = 0
    rejection_counts: Counter[str] = field(default_factory=Counter)


@dataclass(frozen=True)
class TimedFix:
    elapsed_ns: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    accuracy_m: float
    provider: str

    @property
    def position_m(self) -> np.ndarray:
        return lla_to_ecef(self.latitude_deg, self.longitude_deg, self.altitude_m)


@dataclass(frozen=True)
class TimedEpoch:
    elapsed_ns: int
    clock_arc: int
    epoch: Epoch


@dataclass
class ReplayPreprocessResult:
    session_id: str | None
    records: list[dict[str, object]]
    epochs: list[TimedEpoch]
    fused_fixes: list[TimedFix]
    rejection_counts: Counter[str]
    record_counts: Counter[str]
    navigation_health: NavigationHealth
    clock_arcs: list[int]
    structural_validation: dict[str, object]


class GpsL1CaProvider:
    """Causal GPS L1 C/A subframe store and broadcast state evaluator."""

    def __init__(self, maximum_age_s: float = MAX_EPHEMERIS_AGE_S) -> None:
        self.maximum_age_s = maximum_age_s
        self._partial: dict[int, dict[int, tuple[int, dict[str, float | int]]]] = {}
        self._complete: dict[int, BroadcastEphemeris] = {}
        self.health = NavigationHealth()

    def ingest(self, record: dict[str, object]) -> None:
        self.health.received_messages += 1
        reason = self._validate(record)
        if reason is not None:
            self._reject(reason)
            return
        try:
            payload = base64.b64decode(str(record["data_base64"]), validate=True)
            subframe_id = int(record["submessage_id"])
            decoded = decode_subframe(subframe_id, payload)
        except (KeyError, TypeError, ValueError):
            self._reject("decode_failure")
            return
        svid = int(record["svid"])
        elapsed_ns = int(record["elapsed_realtime_nanos"])
        self._partial.setdefault(svid, {})[subframe_id] = (elapsed_ns, decoded)
        self.health.accepted_subframes += 1
        assembled = self._assemble(svid)
        if assembled is not None:
            self._complete[svid] = assembled
            self.health.complete_ephemerides = len(self._complete)

    def state_at(self, svid: int, carrier_frequency_hz: float | None,
                 code_type: str, transmit_time_ns: int) -> tuple[SatelliteState | None, str | None]:
        if carrier_frequency_hz is not None and (
                not math.isfinite(carrier_frequency_hz) or
                abs(carrier_frequency_hz - GPS_L1_HZ) > GPS_L1_TOLERANCE_HZ):
            return None, "unsupported_signal"
        if code_type and code_type.upper() != "C":
            return None, "unsupported_signal"
        ephemeris = self._complete.get(svid)
        if ephemeris is None:
            return None, "unavailable_ephemeris"
        if ephemeris.sv_health != 0:
            return None, "unhealthy_satellite"
        accuracy = ura_accuracy_m(ephemeris.ura_index)
        if not math.isfinite(accuracy):
            return None, "unhealthy_satellite"
        state = evaluate_broadcast_state(ephemeris, transmit_time_ns, accuracy)
        if state is None:
            return None, "invalid_pseudorange"
        if state.source_age_s > self.maximum_age_s:
            return None, "stale_ephemeris"
        return state, None

    def _validate(self, record: dict[str, object]) -> str | None:
        try:
            if int(record["message_type"]) != GPS_L1_CA_TYPE:
                return "unsupported_message_type"
            if int(record["svid"]) not in range(1, 33):
                return "invalid_svid"
            if int(record["status"]) not in (1, 2):
                return "parity_not_verified"
            subframe_id = int(record["submessage_id"])
            if subframe_id not in (1, 2, 3):
                return "unsupported_subframe"
            payload = base64.b64decode(str(record["data_base64"]), validate=True)
            if len(payload) != 40:
                return "invalid_length"
            if unsigned_bits(payload, 49, 3) != subframe_id:
                return "subframe_id_mismatch"
        except (KeyError, TypeError, ValueError):
            return "decode_failure"
        return None

    def _assemble(self, svid: int) -> BroadcastEphemeris | None:
        parts = self._partial[svid]
        if not all(index in parts for index in (1, 2, 3)):
            return None
        elapsed = [parts[index][0] for index in (1, 2, 3)]
        if max(elapsed) - min(elapsed) > MAX_SUBFRAME_SPAN_NS:
            return None
        one, two, three = (parts[index][1] for index in (1, 2, 3))
        if two["iode"] != three["iode"] or int(one["iodc"]) & 0xFF != two["iode"]:
            return None
        return BroadcastEphemeris(
            svid=svid,
            week_modulo_1024=int(one["week"]), ura_index=int(one["ura"]),
            sv_health=int(one["health"]), iodc=int(one["iodc"]), iode=int(two["iode"]),
            tgd_s=float(one["tgd"]), toc_s=float(one["toc"]), af0_s=float(one["af0"]),
            af1_sps=float(one["af1"]), af2_sps2=float(one["af2"]),
            crs_m=float(two["crs"]), delta_n_radps=float(two["delta_n"]),
            m0_rad=float(two["m0"]), cuc_rad=float(two["cuc"]),
            eccentricity=float(two["e"]), cus_rad=float(two["cus"]),
            sqrt_a_sqrt_m=float(two["sqrt_a"]), toe_s=float(two["toe"]),
            cic_rad=float(three["cic"]), omega0_rad=float(three["omega0"]),
            cis_rad=float(three["cis"]), i0_rad=float(three["i0"]),
            crc_m=float(three["crc"]), omega_rad=float(three["omega"]),
            omega_dot_radps=float(three["omega_dot"]), i_dot_radps=float(three["i_dot"]),
        )

    def _reject(self, reason: str) -> None:
        self.health.rejected_messages += 1
        self.health.rejection_counts[reason] += 1


def unsigned_bits(data: bytes, index: int, length: int) -> int:
    if index < 0 or length not in range(1, 33) or index + length > 300:
        raise ValueError("GPS bit range outside 300-bit subframe")
    result = 0
    for offset in range(length):
        logical = index + offset
        word = logical // 30
        physical = logical + (word + 1) * 2
        bit = (data[physical // 8] >> (7 - physical % 8)) & 1
        result = (result << 1) | bit
    return result


def signed_bits(data: bytes, index: int, length: int) -> int:
    raw = unsigned_bits(data, index, length)
    return raw - (1 << length) if raw & (1 << (length - 1)) else raw


def unsigned_split_32(data: bytes, index_8: int, index_24: int) -> int:
    return (unsigned_bits(data, index_8, 8) << 24) | unsigned_bits(data, index_24, 24)


def signed_split_32(data: bytes, index_8: int, index_24: int) -> int:
    raw = unsigned_split_32(data, index_8, index_24)
    return raw - (1 << 32) if raw & (1 << 31) else raw


def decode_subframe(subframe_id: int, data: bytes) -> dict[str, float | int]:
    if len(data) != 40 or unsigned_bits(data, 49, 3) != subframe_id:
        raise ValueError("invalid Android GPS L1 C/A subframe")
    p2 = math.ldexp
    if subframe_id == 1:
        return {
            "week": unsigned_bits(data, 60, 10), "ura": unsigned_bits(data, 72, 4),
            "health": unsigned_bits(data, 76, 6),
            "iodc": (unsigned_bits(data, 82, 2) << 8) | unsigned_bits(data, 210, 8),
            "tgd": signed_bits(data, 196, 8) * p2(1.0, -31),
            "toc": unsigned_bits(data, 218, 16) * p2(1.0, 4),
            "af0": signed_bits(data, 270, 22) * p2(1.0, -31),
            "af1": signed_bits(data, 248, 16) * p2(1.0, -43),
            "af2": signed_bits(data, 240, 8) * p2(1.0, -55),
        }
    if subframe_id == 2:
        return {
            "iode": unsigned_bits(data, 60, 8),
            "crs": signed_bits(data, 68, 16) * p2(1.0, -5),
            "delta_n": signed_bits(data, 90, 16) * p2(1.0, -43) * math.pi,
            "m0": signed_split_32(data, 106, 120) * p2(1.0, -31) * math.pi,
            "cuc": signed_bits(data, 150, 16) * p2(1.0, -29),
            "e": unsigned_split_32(data, 166, 180) * p2(1.0, -33),
            "cus": signed_bits(data, 210, 16) * p2(1.0, -29),
            "sqrt_a": unsigned_split_32(data, 226, 240) * p2(1.0, -19),
            "toe": unsigned_bits(data, 270, 16) * p2(1.0, 4),
        }
    if subframe_id == 3:
        return {
            "cic": signed_bits(data, 60, 16) * p2(1.0, -29),
            "omega0": signed_split_32(data, 76, 90) * p2(1.0, -31) * math.pi,
            "cis": signed_bits(data, 120, 16) * p2(1.0, -29),
            "i0": signed_split_32(data, 136, 150) * p2(1.0, -31) * math.pi,
            "crc": signed_bits(data, 180, 16) * p2(1.0, -5),
            "omega": signed_split_32(data, 196, 210) * p2(1.0, -31) * math.pi,
            "omega_dot": signed_bits(data, 240, 24) * p2(1.0, -43) * math.pi,
            "iode": unsigned_bits(data, 270, 8),
            "i_dot": signed_bits(data, 278, 14) * p2(1.0, -43) * math.pi,
        }
    raise ValueError(f"unsupported subframe {subframe_id}")


def ura_accuracy_m(index: int) -> float:
    if index == 1:
        return 2.8
    if index == 3:
        return 5.7
    if index == 5:
        return 11.3
    if 0 <= index <= 5:
        return math.ldexp(1.0, 1 + index // 2)
    if 6 <= index <= 14:
        return math.ldexp(1.0, index - 2)
    return math.nan


def _normalize_week(seconds: float) -> float:
    return seconds - math.floor((seconds + 302_400.0) / 604_800.0) * 604_800.0


def _resolve_week(modulo_week: int, query_week: int) -> int:
    candidate = query_week - ((query_week - modulo_week) % 1024)
    if query_week - candidate > 512:
        candidate += 1024
    return candidate


def _position_clock(ephemeris: BroadcastEphemeris,
                    gps_seconds: float) -> tuple[np.ndarray, float, float] | None:
    query_week = math.floor(gps_seconds / 604_800.0)
    query_tow = gps_seconds - query_week * 604_800.0
    ephemeris_week = _resolve_week(ephemeris.week_modulo_1024, query_week)
    week_offset = (query_week - ephemeris_week) * 604_800.0
    tc = _normalize_week(week_offset + query_tow - ephemeris.toc_s)
    polynomial = (ephemeris.af0_s + ephemeris.af1_sps * tc +
                  ephemeris.af2_sps2 * tc * tc - ephemeris.tgd_s)
    semi_major = ephemeris.sqrt_a_sqrt_m ** 2
    mean_motion = math.sqrt(MU_M3PS2 / semi_major ** 3) + ephemeris.delta_n_radps
    clock_s = polynomial
    eccentric_anomaly = 0.0
    tk = 0.0
    for _ in range(10):
        tk = _normalize_week(week_offset + query_tow - ephemeris.toe_s - clock_s)
        mean_anomaly = ephemeris.m0_rad + mean_motion * tk
        eccentric_anomaly = mean_anomaly
        converged = False
        for _ in range(20):
            following = mean_anomaly + ephemeris.eccentricity * math.sin(eccentric_anomaly)
            if abs(following - eccentric_anomaly) < 1.0e-13:
                eccentric_anomaly = following
                converged = True
                break
            eccentric_anomaly = following
        if not converged:
            return None
        relativistic = (RELATIVITY_S_PER_SQRT_M * ephemeris.eccentricity *
                        ephemeris.sqrt_a_sqrt_m * math.sin(eccentric_anomaly))
        updated = polynomial + relativistic
        clock_s = updated
    tk = _normalize_week(week_offset + query_tow - ephemeris.toe_s - clock_s)
    true_anomaly = math.atan2(
        math.sqrt(1.0 - ephemeris.eccentricity ** 2) * math.sin(eccentric_anomaly),
        math.cos(eccentric_anomaly) - ephemeris.eccentricity,
    )
    phi = true_anomaly + ephemeris.omega_rad
    twice_phi = 2.0 * phi
    argument = phi + ephemeris.cus_rad * math.sin(twice_phi) + ephemeris.cuc_rad * math.cos(twice_phi)
    radius = (semi_major * (1.0 - ephemeris.eccentricity * math.cos(eccentric_anomaly)) +
              ephemeris.crs_m * math.sin(twice_phi) + ephemeris.crc_m * math.cos(twice_phi))
    inclination = (ephemeris.i0_rad + ephemeris.i_dot_radps * tk +
                   ephemeris.cis_rad * math.sin(twice_phi) + ephemeris.cic_rad * math.cos(twice_phi))
    orbital_x = radius * math.cos(argument)
    orbital_y = radius * math.sin(argument)
    ascending = (ephemeris.omega0_rad + (ephemeris.omega_dot_radps - OMEGA_E_RADPS) * tk -
                 OMEGA_E_RADPS * ephemeris.toe_s)
    position = np.array([
        orbital_x * math.cos(ascending) - orbital_y * math.cos(inclination) * math.sin(ascending),
        orbital_x * math.sin(ascending) + orbital_y * math.cos(inclination) * math.cos(ascending),
        orbital_y * math.sin(inclination),
    ])
    return position, clock_s * C_MPS, tk


def evaluate_broadcast_state(ephemeris: BroadcastEphemeris, gps_time_ns: int,
                             accuracy_m: float | None = None) -> SatelliteState | None:
    if (gps_time_ns <= 0 or not 0.0 <= ephemeris.eccentricity < 1.0 or
            not 4_000.0 <= ephemeris.sqrt_a_sqrt_m <= 6_000.0):
        return None
    seconds = gps_time_ns * 1.0e-9
    center = _position_clock(ephemeris, seconds)
    plus = _position_clock(ephemeris, seconds + 0.5)
    minus = _position_clock(ephemeris, seconds - 0.5)
    if center is None or plus is None or minus is None:
        return None
    return SatelliteState(
        position_m=center[0], velocity_mps=plus[0] - minus[0],
        clock_bias_m=center[1], clock_drift_mps=plus[1] - minus[1],
        source_age_s=abs(center[2]),
        accuracy_m=ura_accuracy_m(ephemeris.ura_index) if accuracy_m is None else accuracy_m,
    )


def read_records(path: Path) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"line {line_number}: invalid JSON: {exc}") from exc
            if not isinstance(record, dict):
                raise ValueError(f"line {line_number}: record is not an object")
            record["_line_number"] = line_number
            records.append(record)
    return records


def _finite_number(record: dict[str, object], key: str, default: float | None = None) -> float | None:
    value = record.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def _latest_fix(fixes: Sequence[TimedFix], elapsed_ns: int, start_index: int) -> tuple[TimedFix | None, int]:
    index = start_index
    while index + 1 < len(fixes) and fixes[index + 1].elapsed_ns <= elapsed_ns:
        index += 1
    if index < len(fixes) and fixes[index].elapsed_ns <= elapsed_ns:
        return fixes[index], index
    return None, index


def preprocess_ndjson(path: Path) -> ReplayPreprocessResult:
    records = read_records(path)
    structural = validate(path, require_all_streams=False)
    record_counts = Counter(str(record.get("record_type")) for record in records)
    session_ids = {str(record["session_id"]) for record in records if "session_id" in record}
    session_id = next(iter(session_ids)) if len(session_ids) == 1 else None
    clocks: dict[int, list[dict[str, object]]] = defaultdict(list)
    raw: dict[int, list[dict[str, object]]] = defaultdict(list)
    navigation: list[dict[str, object]] = []
    fixes: list[TimedFix] = []
    for record in records:
        kind = record.get("record_type")
        elapsed = record.get("elapsed_realtime_nanos")
        if not isinstance(elapsed, int):
            continue
        if kind == "gnss_clock":
            clocks[elapsed].append(record)
        elif kind == "gnss_raw":
            raw[elapsed].append(record)
        elif kind == "gnss_navigation_message":
            navigation.append(record)
        elif kind == "fused_location":
            lat = _finite_number(record, "latitude_deg")
            lon = _finite_number(record, "longitude_deg")
            accuracy = _finite_number(record, "accuracy_m")
            if lat is not None and lon is not None and accuracy is not None:
                fixes.append(TimedFix(
                    elapsed, lat, lon, _finite_number(record, "altitude_m", 0.0) or 0.0,
                    accuracy, str(record.get("provider", "unknown")),
                ))
    navigation.sort(key=lambda value: (int(value["elapsed_realtime_nanos"]), int(value["_line_number"])))
    fixes.sort(key=lambda value: value.elapsed_ns)
    provider = GpsL1CaProvider()
    rejected: Counter[str] = Counter()
    timed_epochs: list[TimedEpoch] = []
    nav_index = 0
    fix_index = 0
    clock_arcs: list[int] = []
    previous_arc: int | None = None

    for elapsed in sorted(set(clocks) | set(raw)):
        while nav_index < len(navigation) and int(navigation[nav_index]["elapsed_realtime_nanos"]) <= elapsed:
            provider.ingest(navigation[nav_index])
            nav_index += 1
        if elapsed not in clocks:
            rejected["orphan_raw_epoch"] += len(raw[elapsed])
            continue
        if len(clocks[elapsed]) != 1:
            rejected["ambiguous_clock_epoch"] += len(raw[elapsed])
            continue
        clock = clocks[elapsed][0]
        measurements = raw.get(elapsed, [])
        if not measurements:
            rejected["clock_without_measurements"] += 1
            continue
        arc_value = clock.get("hardware_clock_discontinuity_count")
        time_nanos = clock.get("time_nanos")
        full_bias = clock.get("full_bias_nanos")
        bias = _finite_number(clock, "bias_nanos", 0.0)
        if not isinstance(arc_value, int) or not isinstance(time_nanos, int) or not isinstance(full_bias, int) or bias is None:
            rejected["missing_clock_bias"] += len(measurements)
            continue
        if previous_arc is None or arc_value != previous_arc:
            clock_arcs.append(arc_value)
            previous_arc = arc_value
        receive_gps_ns = int(time_nanos - full_bias - bias)
        timestamp_ms = receive_gps_ns // 1_000_000
        fix, fix_index = _latest_fix(fixes, elapsed, fix_index)
        receiver = fix.position_m if fix is not None else None
        observations: list[Observation] = []
        for measurement in measurements:
            observation, reason = _prepare_observation(
                measurement, clock, receive_gps_ns, timestamp_ms, receiver, provider,
            )
            if observation is None:
                rejected[reason or "invalid_pseudorange"] += 1
            else:
                observations.append(observation)
        if len(observations) >= 4:
            timed_epochs.append(TimedEpoch(elapsed, arc_value, Epoch(timestamp_ms, tuple(observations))))
        else:
            rejected["insufficient_observations"] += 1

    # Navigation callbacks after the last GNSS epoch still belong in diagnostics.
    while nav_index < len(navigation):
        provider.ingest(navigation[nav_index])
        nav_index += 1
    return ReplayPreprocessResult(
        session_id=session_id, records=records, epochs=timed_epochs, fused_fixes=fixes,
        rejection_counts=rejected, record_counts=record_counts,
        navigation_health=provider.health, clock_arcs=clock_arcs,
        structural_validation=structural,
    )


def _prepare_observation(measurement: dict[str, object], clock: dict[str, object],
                         receive_gps_ns: int, timestamp_ms: int,
                         receiver_m: np.ndarray | None, provider: GpsL1CaProvider
                         ) -> tuple[Observation | None, str | None]:
    constellation = measurement.get("constellation_type")
    svid = measurement.get("svid")
    state = measurement.get("state")
    if constellation != GPS_CONSTELLATION:
        return None, "unsupported_constellation_or_time"
    if not isinstance(svid, int) or not isinstance(state, int):
        return None, "invalid_tracking_state"
    if state & 1 == 0 or state & (8 | 16_384) == 0:
        return None, "invalid_tracking_state"
    frequency = _finite_number(measurement, "carrier_frequency_hz")
    if "carrier_frequency_hz" in measurement and frequency is None:
        return None, "unsupported_frequency"
    cn0 = _finite_number(measurement, "cn0_dbhz")
    if cn0 is None or cn0 < MIN_CN0_DBHZ:
        return None, "low_signal"
    transmit_tow = measurement.get("received_sv_time_nanos")
    transmit_uncertainty = measurement.get("received_sv_time_uncertainty_nanos")
    time_offset = _finite_number(measurement, "time_offset_nanos", 0.0)
    if not isinstance(transmit_tow, int) or not isinstance(transmit_uncertainty, int) or time_offset is None:
        return None, "invalid_time_uncertainty"
    if not 0 <= transmit_uncertainty <= MAX_SV_TIME_UNCERTAINTY_NS:
        return None, "invalid_time_uncertainty"
    observation_receive_ns = receive_gps_ns + time_offset
    week_start = math.floor(observation_receive_ns / GPS_WEEK_NS) * GPS_WEEK_NS
    receive_tow = observation_receive_ns - week_start
    travel_ns = receive_tow - transmit_tow
    if travel_ns > HALF_GPS_WEEK_NS:
        travel_ns -= GPS_WEEK_NS
    if travel_ns < -HALF_GPS_WEEK_NS:
        travel_ns += GPS_WEEK_NS
    raw_pseudorange = travel_ns * (C_MPS * 1.0e-9)
    if not math.isfinite(raw_pseudorange) or not 1.0e6 <= raw_pseudorange <= 1.0e8:
        return None, "invalid_pseudorange"
    transmit_gps_ns = week_start + transmit_tow
    while transmit_gps_ns > observation_receive_ns:
        transmit_gps_ns -= GPS_WEEK_NS
    code_type = str(measurement.get("code_type", ""))
    satellite, reason = provider.state_at(svid, frequency, code_type, int(transmit_gps_ns))
    if satellite is None:
        return None, reason
    elevation_deg = math.nan
    troposphere_m = 0.0
    troposphere_sigma_m = 0.0
    if receiver_m is not None:
        enu = ecef_delta_to_enu(satellite.position_m - receiver_m, receiver_m)
        geometric_range = float(np.linalg.norm(enu))
        if geometric_range < 1.0:
            return None, "missing_correction"
        elevation_rad = math.asin(float(np.clip(enu[2] / geometric_range, -1.0, 1.0)))
        elevation_deg = math.degrees(elevation_rad)
        if elevation_deg < MIN_ELEVATION_DEG:
            return None, "low_elevation"
        mapped_sine = max(math.sin(elevation_rad), 0.10)
        troposphere_m = 2.30 / mapped_sine
        troposphere_sigma_m = 0.75 / mapped_sine
    isb_ns = ((_finite_number(measurement, "full_inter_signal_bias_nanos", 0.0) or 0.0) +
              (_finite_number(measurement, "satellite_inter_signal_bias_nanos", 0.0) or 0.0))
    isb_m = isb_ns * (C_MPS * 1.0e-9)
    corrected_pseudorange = raw_pseudorange + satellite.clock_bias_m - isb_m - troposphere_m
    if not 1.0e6 <= corrected_pseudorange <= 1.0e8:
        return None, "invalid_pseudorange"
    clock_uncertainty_ns = _finite_number(clock, "bias_uncertainty_nanos", 0.0) or 0.0
    transmit_sigma_m = transmit_uncertainty * (C_MPS * 1.0e-9)
    clock_sigma_m = clock_uncertainty_ns * (C_MPS * 1.0e-9)
    atmospheric_sigma_m = math.hypot(7.5, troposphere_sigma_m) if receiver_m is not None else 15.0
    correction_sigma_m = math.hypot(atmospheric_sigma_m, satellite.accuracy_m)
    pseudorange_sigma_m = max(1.5, math.hypot(math.hypot(transmit_sigma_m, clock_sigma_m), correction_sigma_m))
    raw_rate = _finite_number(measurement, "pseudorange_rate_mps")
    rate_sigma = _finite_number(measurement, "pseudorange_rate_uncertainty_mps")
    rate_valid = raw_rate is not None and rate_sigma is not None and rate_sigma > 0.0
    return Observation(
        timestamp_ms=timestamp_ms, constellation=GPS_CONSTELLATION, svid=svid,
        signal_type=code_type or "UNKNOWN", satellite_position_m=satellite.position_m.copy(),
        satellite_velocity_mps=(satellite.velocity_mps.copy() if rate_valid else np.full(3, math.nan)),
        corrected_pseudorange_m=corrected_pseudorange,
        pseudorange_sigma_m=pseudorange_sigma_m,
        corrected_range_rate_mps=(raw_rate + satellite.clock_drift_mps if rate_valid else math.nan),
        range_rate_sigma_mps=max(rate_sigma, 0.15) if rate_valid else math.nan,
        cn0_dbhz=cn0, elevation_deg=elevation_deg,
    ), None


def _motion_metrics(estimates: Sequence[Estimate]) -> dict[str, object]:
    positions = [estimate.position_m for estimate in estimates if np.isfinite(estimate.position_m).all()]
    jumps = [float(np.linalg.norm(positions[index] - positions[index - 1]))
             for index in range(1, len(positions))]
    second = [float(np.linalg.norm(positions[index] - 2.0 * positions[index - 1] + positions[index - 2]))
              for index in range(2, len(positions))]
    return {
        "epochs": len(estimates),
        "solver_failures": sum(not estimate.converged for estimate in estimates),
        "rejected_measurements": sum(estimate.rejected_count for estimate in estimates),
        "mean_observations_per_epoch": (
            float(np.mean([estimate.observation_count for estimate in estimates])) if estimates else None
        ),
        "latency_ms": {
            "mean": float(np.mean([estimate.latency_ms for estimate in estimates])) if estimates else None,
            "max": max((estimate.latency_ms for estimate in estimates), default=None),
        },
        "max_position_jump_m": max(jumps, default=0.0),
        "trajectory_second_difference_rms_m": (
            float(math.sqrt(np.mean(np.square(second)))) if second else 0.0
        ),
    }


def _nearest_fused_baseline(timed_epochs: Sequence[TimedEpoch], fixes: Sequence[TimedFix],
                            estimates: Sequence[Estimate], max_delta_s: float = 2.0
                            ) -> dict[str, object]:
    horizontal: list[float] = []
    vertical: list[float] = []
    deltas: list[float] = []
    for timed, estimate in zip(timed_epochs, estimates):
        if not fixes:
            break
        fix = min(fixes, key=lambda value: abs(value.elapsed_ns - timed.elapsed_ns))
        delta_s = abs(fix.elapsed_ns - timed.elapsed_ns) * 1.0e-9
        if delta_s > max_delta_s:
            continue
        separation = ecef_delta_to_enu(estimate.position_m - fix.position_m, fix.position_m)
        horizontal.append(float(math.hypot(separation[0], separation[1])))
        vertical.append(float(abs(separation[2])))
        deltas.append(delta_s)
    return {
        "interpretation": "separation from Android fused baseline, not ground-truth error",
        "matched_epochs": len(horizontal),
        "maximum_time_delta_s": max_delta_s,
        "mean_time_delta_s": float(np.mean(deltas)) if deltas else None,
        "horizontal_separation_m": {
            "mean": float(np.mean(horizontal)) if horizontal else None,
            "p50": float(np.percentile(horizontal, 50.0)) if horizontal else None,
            "p95": float(np.percentile(horizontal, 95.0)) if horizontal else None,
        },
        "vertical_separation_m": {
            "mean": float(np.mean(vertical)) if vertical else None,
            "p95": float(np.percentile(vertical, 95.0)) if vertical else None,
        },
    }


def _imu_summary(records: Sequence[dict[str, object]]) -> dict[str, object]:
    summary: dict[str, object] = {}
    for sensor in ("ACCELEROMETER", "GYROSCOPE"):
        timestamps = sorted(
            int(record["sensor_timestamp_nanos"])
            for record in records
            if record.get("record_type") == "imu" and record.get("sensor_type") == sensor and
            isinstance(record.get("sensor_timestamp_nanos"), int)
        )
        intervals = np.diff(np.asarray(timestamps, dtype=np.int64)) * 1.0e-9 if len(timestamps) > 1 else np.array([])
        summary[sensor.lower()] = {
            "samples": len(timestamps),
            "duration_s": (timestamps[-1] - timestamps[0]) * 1.0e-9 if len(timestamps) > 1 else 0.0,
            "mean_rate_hz": (1.0 / float(np.mean(intervals))) if len(intervals) and np.mean(intervals) > 0 else None,
            "interval_jitter_std_ms": float(np.std(intervals) * 1.0e3) if len(intervals) else None,
            "non_monotonic_samples": int(sum(interval <= 0 for interval in intervals)),
        }
    return summary


def _write_clean_observations(path: Path, timed_epochs: Sequence[TimedEpoch]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "elapsed_realtime_nanos", "clock_arc", "gps_timestamp_ms", "constellation", "svid",
            "signal_type", "x_sat_ecef_m", "y_sat_ecef_m", "z_sat_ecef_m",
            "vx_sat_ecef_mps", "vy_sat_ecef_mps", "vz_sat_ecef_mps",
            "corrected_pseudorange_m", "pseudorange_sigma_m", "corrected_range_rate_mps",
            "range_rate_sigma_mps", "cn0_dbhz", "elevation_deg",
        ])
        for timed in timed_epochs:
            for observation in timed.epoch.observations:
                writer.writerow([
                    timed.elapsed_ns, timed.clock_arc, timed.epoch.timestamp_ms,
                    observation.constellation, observation.svid, observation.signal_type,
                    *observation.satellite_position_m, *observation.satellite_velocity_mps,
                    observation.corrected_pseudorange_m, observation.pseudorange_sigma_m,
                    observation.corrected_range_rate_mps, observation.range_rate_sigma_mps,
                    observation.cn0_dbhz, observation.elevation_deg,
                ])


def _write_imu(path: Path, records: Sequence[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["elapsed_realtime_nanos", "sensor_type", "x", "y", "z", "accuracy"])
        for record in sorted(
                (item for item in records if item.get("record_type") == "imu"),
                key=lambda item: int(item.get("elapsed_realtime_nanos", -1))):
            values = record.get("values")
            if isinstance(values, list) and len(values) == 3:
                writer.writerow([record.get("elapsed_realtime_nanos"), record.get("sensor_type"),
                                 *values, record.get("accuracy")])


def _write_fused(path: Path, fixes: Sequence[TimedFix]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["elapsed_realtime_nanos", "provider", "latitude_deg", "longitude_deg",
                         "altitude_m", "accuracy_m"])
        for fix in fixes:
            writer.writerow([fix.elapsed_ns, fix.provider, fix.latitude_deg, fix.longitude_deg,
                             fix.altitude_m, fix.accuracy_m])


def _write_solutions(path: Path, timed_epochs: Sequence[TimedEpoch],
                     solutions: Sequence[tuple[str, Sequence[Estimate]]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["elapsed_realtime_nanos", "clock_arc", "gps_timestamp_ms", "estimator",
                         "x_ecef_m", "y_ecef_m", "z_ecef_m", "vx_ecef_mps", "vy_ecef_mps",
                         "vz_ecef_mps", "clock_bias_m", "clock_drift_mps", "iterations",
                         "latency_ms", "converged"])
        for name, estimates in solutions:
            for timed, estimate in zip(timed_epochs, estimates):
                writer.writerow([timed.elapsed_ns, timed.clock_arc, estimate.timestamp_ms, name,
                                 *estimate.position_m, *estimate.velocity_mps, estimate.clock_bias_m,
                                 estimate.clock_drift_mps, estimate.iterations, estimate.latency_ms,
                                 int(estimate.converged)])


def _write_trajectory_svg(path: Path, fixes: Sequence[TimedFix],
                          wls: Sequence[Estimate], fgo: Sequence[Estimate]) -> None:
    reference = fixes[0].position_m if fixes else (
        wls[0].position_m if wls else (fgo[0].position_m if fgo else None)
    )
    if reference is None:
        path.write_text(
            '<svg xmlns="http://www.w3.org/2000/svg" width="760" height="720">\n'
            '<rect width="760" height="720" fill="#f7f8fa"/>\n'
            '<text x="40" y="48" font-family="sans-serif" font-size="20" fill="#151515">'
            'NavGraph replay: no trajectory available</text>\n</svg>\n',
            encoding="utf-8",
        )
        return
    series: list[tuple[str, str, list[np.ndarray]]] = []
    if fixes:
        series.append(("Android fused", "#151515", [
            ecef_delta_to_enu(fix.position_m - reference, reference) for fix in fixes
        ]))
    if wls:
        series.append(("WLS", "#d1495b", [
            ecef_delta_to_enu(estimate.position_m - reference, reference) for estimate in wls
        ]))
    if fgo:
        series.append(("FGO", "#00798c", [
            ecef_delta_to_enu(estimate.position_m - reference, reference) for estimate in fgo
        ]))
    all_points = [point for _, _, points in series for point in points if np.isfinite(point).all()]
    if not all_points:
        _write_trajectory_svg(path, (), (), ())
        return
    east = [float(point[0]) for point in all_points]
    north = [float(point[1]) for point in all_points]
    min_e, max_e, min_n, max_n = min(east), max(east), min(north), max(north)
    span = max(max_e - min_e, max_n - min_n, 1.0)

    def screen(point: np.ndarray) -> tuple[float, float]:
        return (70.0 + 600.0 * (float(point[0]) - min_e) / span,
                650.0 - 600.0 * (float(point[1]) - min_n) / span)

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="800" height="720" viewBox="0 0 800 720">',
        '<rect width="800" height="720" fill="#f7f8fa"/>',
        '<text x="40" y="36" font-family="sans-serif" font-size="20" fill="#151515">NavGraph local ENU trajectory</text>',
        '<rect x="70" y="50" width="600" height="600" fill="#ffffff" stroke="#ccd1d7"/>',
    ]
    for index, (name, color, points) in enumerate(series):
        coordinates = " ".join(
            f"{x:.2f},{y:.2f}" for x, y in map(screen, (point for point in points if np.isfinite(point).all()))
        )
        if coordinates:
            lines.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="3"/>')
        lines.append(f'<circle cx="704" cy="{80 + index * 28}" r="6" fill="{color}"/>')
        lines.append(
            f'<text x="718" y="{86 + index * 28}" font-family="sans-serif" font-size="13">{name}</text>'
        )
    lines.extend([
        '<text x="285" y="695" font-family="sans-serif" font-size="13">East (equal scale with North)</text>',
        '</svg>',
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def replay(path: Path, output_dir: Path, lag_seconds: float = 2.0) -> dict[str, object]:
    result = preprocess_ndjson(path)
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_clean_observations(output_dir / "clean_observations.csv", result.epochs)
    _write_imu(output_dir / "imu.csv", result.records)
    _write_fused(output_dir / "fused_location.csv", result.fused_fixes)
    epochs = [timed.epoch for timed in result.epochs]
    summary: dict[str, object] = {
        "input": str(path), "input_schema": "navgraph_ndjson_v1",
        "session_id": result.session_id, "structural_validation": result.structural_validation,
        "record_counts": dict(sorted(result.record_counts.items())),
        "clock_arcs": result.clock_arcs, "clock_arc_count": len(result.clock_arcs),
        "raw_gnss_callback_epochs": len({
            int(record["elapsed_realtime_nanos"]) for record in result.records
            if record.get("record_type") == "gnss_raw" and isinstance(record.get("elapsed_realtime_nanos"), int)
        }),
        "clean_gnss_epochs": len(epochs),
        "accepted_observations": sum(len(epoch.observations) for epoch in epochs),
        "preprocessing_rejections": dict(sorted(result.rejection_counts.items())),
        "navigation": {
            "received_messages": result.navigation_health.received_messages,
            "accepted_subframes": result.navigation_health.accepted_subframes,
            "rejected_messages": result.navigation_health.rejected_messages,
            "complete_ephemerides": result.navigation_health.complete_ephemerides,
            "rejection_counts": dict(sorted(result.navigation_health.rejection_counts.items())),
        },
        "imu": _imu_summary(result.records), "fused_fix_count": len(result.fused_fixes),
        "lag_seconds": lag_seconds,
    }
    if epochs:
        wls = run_wls(epochs)
        fgo = run_fixed_lag_fgo(epochs, wls, lag_seconds)
        state = np.zeros(8)
        state[:3] = wls[0].position_m
        state[6] = wls[0].clock_bias_m
        jacobians = check_factor_jacobians(state, epochs[0].observations[0])
        _write_solutions(output_dir / "solutions.csv", result.epochs, (("wls", wls), ("fgo", fgo)))
        summary["solver_status"] = "completed"
        summary["jacobian_max_abs_error"] = jacobians
        summary["wls"] = _motion_metrics(wls)
        summary["fgo"] = _motion_metrics(fgo)
        summary["wls_fused_baseline"] = _nearest_fused_baseline(result.epochs, result.fused_fixes, wls)
        summary["fgo_fused_baseline"] = _nearest_fused_baseline(result.epochs, result.fused_fixes, fgo)
        _write_trajectory_svg(output_dir / "trajectory.svg", result.fused_fixes, wls, fgo)
    else:
        _write_solutions(output_dir / "solutions.csv", [], ())
        _write_trajectory_svg(output_dir / "trajectory.svg", result.fused_fixes, (), ())
        summary["solver_status"] = "not_run_no_clean_epochs"
        summary["solver_note"] = (
            "Raw records were parsed, but no epoch had four solver-ready GPS L1 C/A observations. "
            "Inspect preprocessing_rejections and navigation diagnostics."
        )
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--lag-seconds", type=float, default=2.0)
    args = parser.parse_args()
    if args.lag_seconds <= 0.0:
        parser.error("--lag-seconds must be positive")
    summary = replay(args.log, args.output, args.lag_seconds)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
