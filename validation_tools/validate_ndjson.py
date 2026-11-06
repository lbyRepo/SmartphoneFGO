#!/usr/bin/env python3
"""Strict structural and timestamp validation for NavGraph NDJSON logs."""

from __future__ import annotations

import argparse
import base64
import binascii
import json
from collections import Counter
from pathlib import Path


REQUIRED_FIELDS = {
    "session_start": {"app_version", "requested_imu_period_us"},
    "session_end": {"reason"},
    "sensor_meta": {"sensor_type", "sensor_name", "min_delay_us"},
    "imu": {"sensor_type", "sensor_timestamp_nanos", "callback_elapsed_realtime_nanos", "values"},
    "gnss_clock": {"time_nanos", "hardware_clock_discontinuity_count", "callback_elapsed_realtime_nanos"},
    "gnss_raw": {
        "gnss_event_elapsed_realtime_nanos", "callback_elapsed_realtime_nanos",
        "constellation_type", "svid", "state", "received_sv_time_nanos", "cn0_dbhz",
        "pseudorange_rate_mps",
    },
    "gnss_navigation_message": {
        "callback_elapsed_realtime_nanos", "svid", "message_type", "message_id",
        "submessage_id", "status", "data_base64",
    },
    "fused_location": {
        "provider", "latitude_deg", "longitude_deg", "location_time_millis",
        "location_elapsed_realtime_nanos", "callback_elapsed_realtime_nanos",
    },
}


def validate(path: Path, require_all_streams: bool = True) -> dict[str, object]:
    counts: Counter[str] = Counter()
    errors: list[str] = []
    session_id: str | None = None
    last_event_time = -1

    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                errors.append(f"line {line_number}: invalid JSON: {exc}")
                continue
            record_type = record.get("record_type")
            if record_type not in REQUIRED_FIELDS:
                errors.append(f"line {line_number}: unknown record_type {record_type!r}")
                continue
            counts[record_type] += 1
            common = {"schema_version", "record_type", "session_id", "elapsed_realtime_nanos", "wall_time_millis"}
            missing = (common | REQUIRED_FIELDS[record_type]) - record.keys()
            if missing:
                errors.append(f"line {line_number}: missing {sorted(missing)}")
            if record.get("schema_version") != 1:
                errors.append(f"line {line_number}: unsupported schema_version")
            current_session = record.get("session_id")
            if session_id is None:
                session_id = current_session
            elif current_session != session_id:
                errors.append(f"line {line_number}: session_id changed")
            event_time = record.get("elapsed_realtime_nanos")
            if not isinstance(event_time, int) or event_time < 0:
                errors.append(f"line {line_number}: invalid elapsed_realtime_nanos")
            elif event_time + 5_000_000_000 < last_event_time:
                errors.append(f"line {line_number}: event time regressed by more than 5 seconds")
            else:
                last_event_time = max(last_event_time, event_time)

            callback_time = record.get("callback_elapsed_realtime_nanos")
            if isinstance(callback_time, int) and isinstance(event_time, int) and callback_time < event_time:
                errors.append(f"line {line_number}: callback precedes event")
            if record_type == "imu" and record.get("sensor_timestamp_nanos") != event_time:
                errors.append(f"line {line_number}: IMU event time is not sensor timestamp")
            if record_type == "gnss_raw" and record.get("gnss_event_elapsed_realtime_nanos") != event_time:
                errors.append(f"line {line_number}: raw GNSS event time mismatch")
            if record_type == "gnss_clock" and "elapsed_realtime_nanos_from_clock" in record:
                if record["elapsed_realtime_nanos_from_clock"] != event_time:
                    errors.append(f"line {line_number}: GNSS clock event time mismatch")
            if record_type == "gnss_navigation_message":
                if callback_time != event_time:
                    errors.append(f"line {line_number}: navigation message event time mismatch")
                try:
                    payload = base64.b64decode(record.get("data_base64", ""), validate=True)
                except (binascii.Error, ValueError):
                    errors.append(f"line {line_number}: invalid navigation message Base64")
                else:
                    if not payload:
                        errors.append(f"line {line_number}: empty navigation message payload")
                    if record.get("message_type") == 257 and len(payload) != 40:
                        errors.append(f"line {line_number}: GPS L1 C/A payload must be 40 bytes")
            if record_type == "fused_location" and record.get("location_elapsed_realtime_nanos") != event_time:
                errors.append(f"line {line_number}: location event time mismatch")

    if counts["session_start"] != 1 or counts["session_end"] != 1:
        errors.append("log must contain exactly one session_start and one session_end")
    if require_all_streams:
        for record_type in ("imu", "gnss_clock", "gnss_raw", "fused_location"):
            if counts[record_type] == 0:
                errors.append(f"required stream missing: {record_type}")
    result = {
        "path": str(path),
        "session_id": session_id,
        "record_counts": dict(sorted(counts.items())),
        "valid": not errors,
        "errors": errors[:100],
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--allow-missing-streams", action="store_true")
    args = parser.parse_args()
    result = validate(args.log, require_all_streams=not args.allow_missing_streams)
    print(json.dumps(result, indent=2))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
