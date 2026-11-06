# S23 GNSS Logger NDJSON Schema

The Android logger writes newline-delimited JSON (`.ndjson`). Each line is one complete JSON object with a shared envelope and a `record_type`-specific payload. The format is append-only friendly and can be parsed incrementally by `python_tools/replay/parse_log.py`.

## Common fields

All records include:

| Field | Type | Description |
| --- | --- | --- |
| `schema_version` | integer | Current value: `1`. |
| `record_type` | string | One of the record types below. |
| `session_id` | string | UTC timestamp plus random suffix generated when logging starts. |
| `elapsed_realtime_nanos` | integer | Monotonic timestamp associated with the record. For many current records this is callback/write receipt time; record-specific event timestamps must be preferred when present. |
| `wall_time_millis` | integer | `System.currentTimeMillis()` for file correlation only; estimator code should prefer monotonic timestamps. |

## Record types

### `session_start`

Written once when logging starts.

Required fields: `app_version`, `android_sdk`, `device_manufacturer`, `device_model`, `device`, `hardware`, `fingerprint`, `requested_imu_period_us`.

### `session_stop`

Written once when logging stops normally.

Required fields: `reason`.

### `sensor_meta`

Written for each relevant Android sensor discovered at startup.

Required fields: `sensor_type`, `sensor_name`, `vendor`, `version`, `resolution`, `maximum_range`, `power_ma`, `min_delay_us`, `max_delay_us`, `fifo_reserved_event_count`, `fifo_max_event_count`, `reporting_mode`, `wake_up`.

### `imu`

Accelerometer and gyroscope samples. The logger records calibrated sensors when available and falls back to default accelerometer/gyroscope.

Required fields: `sensor_type`, `sensor_name`, `sensor_timestamp_nanos`, `accuracy`, `values`.

`values` is a three-number array. Accelerometer units are `m/s^2`; gyroscope units are `rad/s`.

### `gnss_clock`

One clock record is written for each `GnssMeasurementsEvent`.

Required fields: `time_nanos`, `hardware_clock_discontinuity_count`.

Optional fields are present only when Android reports them: `leap_second`, `time_uncertainty_nanos`, `full_bias_nanos`, `bias_nanos`, `bias_uncertainty_nanos`, `drift_nanos_per_second`, `drift_uncertainty_nanos_per_second`, `elapsed_realtime_nanos_from_clock`, `elapsed_realtime_uncertainty_nanos`.

Important: `elapsed_realtime_nanos_from_clock`, when present, is the preferred GNSS event timestamp for synchronization. The common `elapsed_realtime_nanos` field may be callback receipt time and must not be treated as GNSS measurement time without checking the source.

### `gnss_raw`

One record per `GnssMeasurement`.

Required fields: `gnss_event_elapsed_realtime_nanos`, `constellation_type`, `svid`, `state`, `received_sv_time_nanos`, `received_sv_time_uncertainty_nanos`, `cn0_dbhz`, `pseudorange_rate_mps`, `pseudorange_rate_uncertainty_mps`.

Optional fields are present only when Android reports them: `carrier_frequency_hz`, `code_type`, `baseband_cn0_dbhz`, `automatic_gain_control_db`, `accumulated_delta_range_m`, `accumulated_delta_range_uncertainty_m`, `accumulated_delta_range_state`, `multipath_indicator`, `satellite_inter_signal_bias_nanos`, `satellite_inter_signal_bias_uncertainty_nanos`, `full_inter_signal_bias_nanos`, `full_inter_signal_bias_uncertainty_nanos`.

### `fused_location`

Baseline Android fused location, using the platform fused provider when present.

Required fields: `provider`, `location_elapsed_realtime_nanos`, `latitude_deg`, `longitude_deg`, `time_millis`.

Optional fields are present only when Android reports them: `altitude_m`, `speed_mps`, `bearing_deg`, `horizontal_accuracy_m`, `vertical_accuracy_m`, `speed_accuracy_mps`, `bearing_accuracy_deg`, `mock`.

## Timestamp policy

Use monotonic nanoseconds for fusion/replay ordering:

- `imu.sensor_timestamp_nanos` is the sensor event timestamp on the `elapsedRealtimeNanos()` time base.
- `fused_location.location_elapsed_realtime_nanos` is the Android location fix timestamp on the same monotonic time base.
- `gnss_clock.time_nanos` and GNSS measurement satellite times are GNSS-clock-domain fields and must be interpreted with the matching clock record.
- `gnss_clock.elapsed_realtime_nanos_from_clock`, when present, is the preferred GNSS event time.
- `elapsed_realtime_nanos` records callback receipt/write time unless a collector explicitly documents otherwise; it is useful for latency checks.

`wall_time_millis` exists only for human correlation and file/session naming.

## Versioning

`schema_version = 1` is still prototype format. Any incompatible field rename, timestamp semantic change, or unit change must either preserve backward parsing or bump the schema version and update tests.
