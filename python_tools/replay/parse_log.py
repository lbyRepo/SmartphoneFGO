#!/usr/bin/env python3
"""Parse and lightly validate S23 GNSS Logger NDJSON files."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any


COMMON_REQUIRED = {
    "schema_version",
    "record_type",
    "session_id",
    "elapsed_realtime_nanos",
    "wall_time_millis",
}

TYPE_REQUIRED = {
    "session_start": {
        "app_version",
        "android_sdk",
        "device_manufacturer",
        "device_model",
        "device",
        "hardware",
        "fingerprint",
        "requested_imu_period_us",
    },
    "session_stop": {"reason"},
    "sensor_meta": {
        "sensor_type",
        "sensor_name",
        "vendor",
        "version",
        "resolution",
        "maximum_range",
        "power_ma",
        "min_delay_us",
        "max_delay_us",
        "fifo_reserved_event_count",
        "fifo_max_event_count",
        "reporting_mode",
        "wake_up",
    },
    "imu": {"sensor_type", "sensor_name", "sensor_timestamp_nanos", "accuracy", "values"},
    "gnss_clock": {"time_nanos", "hardware_clock_discontinuity_count"},
    "gnss_raw": {
        "gnss_event_elapsed_realtime_nanos",
        "constellation_type",
        "svid",
        "state",
        "received_sv_time_nanos",
        "received_sv_time_uncertainty_nanos",
        "cn0_dbhz",
        "pseudorange_rate_mps",
        "pseudorange_rate_uncertainty_mps",
    },
    "fused_location": {
        "provider",
        "location_elapsed_realtime_nanos",
        "latitude_deg",
        "longitude_deg",
        "time_millis",
    },
}


def validate_record(record: dict[str, Any], line_no: int) -> list[str]:
    errors: list[str] = []
    missing_common = sorted(COMMON_REQUIRED - record.keys())
    if missing_common:
        errors.append(f"line {line_no}: missing common fields {missing_common}")
        return errors

    if record["schema_version"] != 1:
        errors.append(f"line {line_no}: unsupported schema_version {record['schema_version']!r}")

    record_type = record["record_type"]
    required = TYPE_REQUIRED.get(record_type)
    if required is None:
        errors.append(f"line {line_no}: unknown record_type {record_type!r}")
        return errors

    missing = sorted(required - record.keys())
    if missing:
        errors.append(f"line {line_no}: {record_type} missing fields {missing}")

    if record_type == "imu":
        values = record.get("values")
        if not isinstance(values, list) or len(values) != 3:
            errors.append(f"line {line_no}: imu.values must be a 3-element list")

    return errors


def parse_file(path: Path) -> tuple[Counter[str], list[str]]:
    counts: Counter[str] = Counter()
    errors: list[str] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, 1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                record = json.loads(stripped)
            except json.JSONDecodeError as exc:
                errors.append(f"line {line_no}: invalid JSON: {exc}")
                continue
            if not isinstance(record, dict):
                errors.append(f"line {line_no}: record must be a JSON object")
                continue
            counts[str(record.get("record_type", "<missing>"))] += 1
            errors.extend(validate_record(record, line_no))
    return counts, errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path, help="Path to .ndjson log exported by the Android app")
    parser.add_argument("--strict", action="store_true", help="Return non-zero when validation warnings are found")
    args = parser.parse_args()

    counts, errors = parse_file(args.log)
    print("record_counts")
    for record_type, count in sorted(counts.items()):
        print(f"  {record_type}: {count}")

    if errors:
        print("validation_errors", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        return 1 if args.strict else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
