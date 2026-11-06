# Module Ownership Map

## `android_app/`

Owner role: Android sensor logging agent.

Responsibilities:

- Runtime permission handling.
- Foreground service lifecycle.
- Android Raw GNSS callback registration.
- GNSS clock and measurement serialization.
- Accelerometer and gyroscope collection.
- Fused location baseline collection.
- On-device NDJSON file writing.

Inputs: Android framework callbacks, device sensors, location providers.

Outputs: NDJSON logs conforming to `docs/log_schema.md`.

Must not own: estimator math, satellite orbit computation, FGO/EKF state updates, offline plotting.

## `cpp_core/`

Owner role: C++ estimator core agent.

Responsibilities:

- Stable estimator-facing data types.
- Coordinate/time/status abstractions.
- GNSS preprocessing interfaces and coordinate transforms.
- Future GNSS preprocessing, INS propagation, WLS, EKF, and FGO implementations.
- Android-independent APIs callable from JNI or desktop replay.

Inputs: parsed log records, ephemeris/corrections once available, IMU samples, GNSS measurement epochs.

Outputs: navigation states, covariances, quality flags, diagnostics.

Must not own: Android permissions/UI, file-picker UX, Python plotting.

## `python_tools/`

Owner role: replay and evaluation agent.

Responsibilities:

- Parse and validate NDJSON logs.
- Convert raw GNSS replay records into clean observation epochs.
- Convert Android logs into estimator input streams.
- Import future GSDC datasets.
- Compute accuracy/runtime metrics.
- Produce plots and field-test reports.

Inputs: exported logs, fixtures, future ground truth tracks.

Outputs: validation summaries, metrics tables, plots, replay streams.

Must not own: Android runtime collection code or production estimator math.

## `tests/`

Owner role: verification agent.

Responsibilities:

- Maintain compact fixtures for schema and replay validation.
- Add Android instrumentation tests when app behavior needs automation.
- Add C++ unit tests before estimator implementation grows.
- Add replay regression tests for real S23+ logs once available.

## `docs/`

Owner role: project architect agent.

Responsibilities:

- Public schemas.
- Module boundaries.
- Interface contracts.
- Roadmap and milestone checklists.
- Test strategy and acceptance criteria.
