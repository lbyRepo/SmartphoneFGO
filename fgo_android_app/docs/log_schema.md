# Log schema

## Format and versioning

NavGraph writes UTF-8 newline-delimited JSON (NDJSON): one complete JSON object per
line and one session per file. The current schema version is `1`.

Files are written to the app-specific external Documents directory when available:

`Android/data/com.example.navgraph/files/Documents/logs/navgraph_<session_id>.ndjson`

Optional Android fields are omitted when the source API reports them unavailable.
They are never written as zero, `null`, or `NaN` placeholders. Unknown fields must
be ignored by version-1 readers. Removing a required field or changing its meaning,
type, or unit requires a new schema version.

## Common envelope

Every record contains:

| Field | Type | Unit | Meaning |
|---|---|---|---|
| `schema_version` | integer | - | Always `1` for this document. |
| `record_type` | string | - | One of the record types below. |
| `session_id` | string | - | Stable ID shared by every line in a file. |
| `elapsed_realtime_nanos` | integer | ns | Primary monotonic event time for ordering/alignment. |
| `wall_time_millis` | integer | ms since Unix epoch | Write-time metadata; not an integration clock. |

`elapsed_realtime_nanos` uses the Android elapsed-realtime time base, including
deep sleep. It has a record-specific source described below.

## Record types

### `session_start`

Exactly one, first in a normally closed session.

Required payload fields:

| Field | Type | Meaning |
|---|---|---|
| `app_version` | string | Application version that produced the file. |
| `android_sdk` | integer | Android API level. |
| `device_manufacturer` | string | `Build.MANUFACTURER`. |
| `device_model` | string | `Build.MODEL`. |
| `device` | string | `Build.DEVICE`. |
| `hardware` | string | `Build.HARDWARE`. |
| `fingerprint` | string | Android build fingerprint. |
| `requested_imu_period_us` | integer | Requested sampling period, currently 10,000 us. |

Its event time is sampled from `SystemClock.elapsedRealtimeNanos()`.

### `session_end`

Exactly one in a cleanly stopped session.

| Field | Type | Meaning |
|---|---|---|
| `reason` | string | Stop reason such as `user_stop`, `service_destroyed`, or `closed`. |

An absent `session_end` identifies a truncated or unclean session and must not make
otherwise parseable preceding records disappear.

### `sensor_meta`

Written once per registered accelerometer or gyroscope at session start.

| Field | Type | Unit/meaning |
|---|---|---|
| `sensor_type` | string | `ACCELEROMETER` or `GYROSCOPE`. |
| `sensor_name` | string | Android sensor name. |
| `vendor` | string | Sensor vendor. |
| `version` | integer | Sensor implementation version. |
| `resolution` | number | Smallest reported change in the sensor's native units. |
| `maximum_range` | number | Maximum range in native units. |
| `power_ma` | number | Estimated current in mA. |
| `min_delay_us` | integer | Minimum supported reporting delay. |
| `max_delay_us` | integer | Maximum reporting delay. |
| `fifo_reserved_event_count` | integer | Reserved FIFO capacity. |
| `fifo_max_event_count` | integer | Maximum FIFO capacity. |
| `reporting_mode` | integer | Android reporting mode constant. |
| `wake_up` | boolean | Whether this is a wake-up sensor. |

### `imu`

One record per Android `SensorEvent`.

| Field | Type | Unit/meaning |
|---|---|---|
| `sensor_type` | string | `ACCELEROMETER` or `GYROSCOPE`. |
| `sensor_name` | string | Source sensor name. |
| `sensor_timestamp_nanos` | integer | Sensor event time in elapsed-realtime ns. |
| `callback_elapsed_realtime_nanos` | integer | Callback-observed elapsed-realtime ns. |
| `accuracy` | integer | Android sensor accuracy constant. |
| `values` | array[number] | Three values in Android device-axis order. |

For accelerometer records, `values` are specific force in m/s2. For gyroscope
records, they are angular rate in rad/s. The common
`elapsed_realtime_nanos` must equal `sensor_timestamp_nanos`.

### `gnss_clock`

One record per `GnssMeasurementsEvent`, before that event's `gnss_raw` records.

Required fields:

| Field | Type | Unit/meaning |
|---|---|---|
| `time_nanos` | integer | Receiver hardware clock value in ns. |
| `hardware_clock_discontinuity_count` | integer | Android discontinuity counter. |
| `callback_elapsed_realtime_nanos` | integer | Callback-observed elapsed-realtime ns. |

Optional fields, present only when the matching `GnssClock.has*()` is true:

| Field | Type | Unit |
|---|---|---|
| `leap_second` | integer | seconds |
| `time_uncertainty_nanos` | number | ns |
| `full_bias_nanos` | integer | ns |
| `bias_nanos` | number | ns |
| `bias_uncertainty_nanos` | number | ns |
| `drift_nanos_per_second` | number | ns/s |
| `drift_uncertainty_nanos_per_second` | number | ns/s |
| `elapsed_realtime_nanos_from_clock` | integer | ns |
| `elapsed_realtime_uncertainty_nanos` | number | ns |

When `elapsed_realtime_nanos_from_clock` exists, it is the common event time.
Otherwise callback elapsed time is used. A changed discontinuity count divides the
log into separate GNSS clock arcs.

### `gnss_raw`

One record per `GnssMeasurement` in an event.

Required fields:

| Field | Type | Unit/meaning |
|---|---|---|
| `gnss_event_elapsed_realtime_nanos` | integer | Parent GNSS event monotonic time. |
| `callback_elapsed_realtime_nanos` | integer | Callback-observed elapsed-realtime ns. |
| `constellation_type` | integer | Android `GnssStatus.CONSTELLATION_*` value. |
| `svid` | integer | Space vehicle identifier in Android's constellation convention. |
| `state` | integer | `GnssMeasurement.STATE_*` bit mask. |
| `received_sv_time_nanos` | integer | Received satellite transmit time in ns. |
| `received_sv_time_uncertainty_nanos` | integer | Transmit-time uncertainty in ns. |
| `cn0_dbhz` | number | C/N0 in dB-Hz. |
| `pseudorange_rate_mps` | number | Android pseudorange rate in m/s. |
| `pseudorange_rate_uncertainty_mps` | number | 1-sigma rate uncertainty in m/s. |
| `accumulated_delta_range_state` | integer | `ADR_STATE_*` bit mask. |
| `accumulated_delta_range_m` | number | Accumulated delta range in m. |
| `accumulated_delta_range_uncertainty_m` | number | 1-sigma ADR uncertainty in m. |
| `multipath_indicator` | integer | Android multipath indicator. |
| `code_type` | string | Android signal code type. |

Optional fields:

| Field | Type | Unit |
|---|---|---|
| `time_offset_nanos` | number | Measurement receive-time offset relative to the event clock in ns. |
| `carrier_frequency_hz` | number | Hz |
| `full_inter_signal_bias_nanos` | number | ns |
| `full_inter_signal_bias_uncertainty_nanos` | number | ns |
| `satellite_inter_signal_bias_nanos` | number | ns |
| `satellite_inter_signal_bias_uncertainty_nanos` | number | ns |
| `baseband_cn0_dbhz` | number | dB-Hz |
| `automatic_gain_control_level_db` | number | dB |

`time_offset_nanos` was added additively in app 0.6.0; older schema-v1 records may
omit it and replay readers must use `0.0` only for those older records. The live
pseudorange preprocessor always receives the Android value and includes it in
receive-time reconstruction.

The common event time must equal `gnss_event_elapsed_realtime_nanos`. All raw
records sharing that value with the immediately preceding clock record form one
GNSS callback epoch. This schema stores ingredients for pseudorange preparation;
it does not claim that `received_sv_time_nanos` is itself a pseudorange.

### `gnss_navigation_message`

One record per `GnssNavigationMessage` callback. Navigation-message support is
optional hardware behavior, so a valid session may contain zero records.

| Field | Type | Unit/meaning |
|---|---|---|
| `callback_elapsed_realtime_nanos` | integer | Callback-observed elapsed-realtime ns. |
| `svid` | integer | Satellite identifier in Android's constellation convention. |
| `message_type` | integer | Android `GnssNavigationMessage.TYPE_*` value. |
| `message_id` | integer | Message/page identifier defined by the Android message type. |
| `submessage_id` | integer | Subframe/string/word identifier defined by the message type. |
| `status` | integer | Android parity status: unknown, passed, or rebuilt. |
| `data_base64` | string | Exact Android message bytes encoded as unwrapped RFC 4648 Base64. |

The API provides no separate source monotonic timestamp, so both the common event
time and callback time use the same `elapsedRealtimeNanos()` sample. For GPS L1
C/A (`message_type=257`), the decoded payload must be 40 bytes: ten big-endian
30-bit words, each stored in the low 30 bits of a four-byte word. Logs preserve all
message types; app 0.7.0 only decodes parity-verified GPS L1 C/A subframes 1-3.

### `fused_location`

One record per new provider callback. No stale last-known fixes are written.

Required fields:

| Field | Type | Unit/meaning |
|---|---|---|
| `provider` | string | Usually `fused`, `gps`, or `network`. |
| `latitude_deg` | number | WGS84 degrees. |
| `longitude_deg` | number | WGS84 degrees. |
| `location_time_millis` | integer | Provider wall/UTC time in Unix ms. |
| `location_elapsed_realtime_nanos` | integer | Provider monotonic event time in ns. |
| `callback_elapsed_realtime_nanos` | integer | Callback-observed elapsed-realtime ns. |
| `accuracy_m` | number | Horizontal 68-percent radius reported by Android. |

Optional fields:

| Field | Type | Unit |
|---|---|---|
| `altitude_m` | number | m |
| `vertical_accuracy_m` | number | m |
| `speed_mps` | number | m/s |
| `speed_accuracy_mps` | number | m/s |
| `bearing_deg` | number | degrees clockwise from true north |
| `bearing_accuracy_deg` | number | degrees |

The common event time must equal `location_elapsed_realtime_nanos`.

## Ordering and synchronization

Writes are serialized, but collectors run on different threads. File order is
write order, not a guarantee that event timestamps strictly increase globally.
Readers must sort within a stream by monotonic event time and tolerate bounded
cross-stream reordering. They must not sort IMU samples by wall time.

For latency diagnostics, compute callback time minus source event time. A negative
value indicates invalid source data or a time-base misuse and must be reported.

For GNSS pseudorange preparation, use `GnssClock` and constellation-specific
transmit-time rules. Do not use elapsed-realtime or wall time as GNSS receive time.
At minimum, require the relevant tracking-state bits, usable clock bias fields, a
valid transmit-time uncertainty, and an ephemeris solution for the transmit epoch.

## Example

```json
{"schema_version":1,"record_type":"imu","session_id":"20260716_010203_ab12cd34","elapsed_realtime_nanos":123456789000,"wall_time_millis":1784163723000,"sensor_type":"GYROSCOPE","sensor_name":"Example Gyroscope","sensor_timestamp_nanos":123456789000,"callback_elapsed_realtime_nanos":123456900000,"accuracy":3,"values":[0.001,-0.002,0.003]}
```

## Validation

From the workspace root:

```bash
python3 validation_tools/validate_ndjson.py device_logs/navgraph_<session_id>.ndjson
```

Use `--allow-missing-streams` only for diagnostics that intentionally lack one or
more streams. A complete acceptance log requires `imu`, `gnss_clock`, `gnss_raw`,
and `fused_location`, plus exactly one session start and end.
