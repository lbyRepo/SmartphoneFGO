# AGENTS.md

This file defines the engineering rules for changes under `fgo_android_app`.
The app is a research prototype, so reproducibility and honest boundaries matter
more than optimistic accuracy claims.

Follow the repository-level Git and commit proposal rules in `../AGENTS.md`.
Do not create commits. Every handoff that changes files must propose a Conventional
Commits title (`feat`, `fix`, `chore`, `docs`, or another appropriate type), plus a
description of the changes and verification. State whether changes are staged.

## Project scope

- `app/src/main/java/com/example/navgraph/logging` owns Android permissions,
  foreground collection, and schema-v1 NDJSON output.
- `app/src/main/java/com/example/navgraph/estimator` owns portable estimator logic,
  coordinate transforms, synthetic data, and replay models.
- `app/src/main/java/com/example/navgraph/ui` renders state and diagnostics. It must
  not contain measurement or estimator math.
- `app/src/main/assets/gsdc_2023_sample` is a small deterministic public regression
  fixture, not a field-accuracy dataset.
- `docs` records contracts and verification evidence. Update the relevant document
  whenever a public record, frame, timestamp, or estimator interface changes.
- `../validation_tools` contains desktop reference and log-validation tools.

Do not edit generated files under `build` or `app/build`.

## Non-negotiable contracts

### Time

- Use monotonic nanoseconds for ordering and integration.
- IMU propagation uses `SensorEvent.timestamp`.
- A raw GNSS epoch uses `GnssClock.elapsedRealtimeNanos` when present and retains
  callback time separately.
- Fused locations use `Location.elapsedRealtimeNanos` for event ordering.
- `wall_time_millis`, `Location.time`, and GNSS time are metadata or conversion
  inputs. They are not substitutes for monotonic integration time.
- Detect non-increasing IMU timestamps and GNSS hardware clock discontinuities.
  Never silently bridge those discontinuities.

### Frames and units

- Satellite and absolute receiver states are WGS84 ECEF in meters and meters per
  second.
- Inertial states are local ENU relative to an explicit ECEF origin.
- `attitudeBodyToEnu` rotates vectors from Android device coordinates into ENU.
- Android accelerometer samples are specific force in m/s2; gyroscope samples are
  angular rate in rad/s.
- Receiver clock bias is meters and clock drift is meters per second inside the
  estimator. Raw Android clock fields remain in nanoseconds in logs.
- Put units in field and variable names. Do not pass anonymous `DoubleArray`
  values across a module boundary without a typed wrapper or documented order.

### GNSS

- Raw `GnssMeasurement` data does not include satellite ECEF states. Live
  positioning requires an ephemeris/correction provider before creating a clean
  `GnssObservation`.
- Preserve constellation, SVID, signal/frequency, state flags, uncertainty, and
  clock discontinuity information through preprocessing.
- Reject invalid inputs with stable reason codes and count every rejection.
- Do not infer carrier-phase usability from ADR presence alone. Respect ADR reset
  and cycle-slip flags and keep TDCP/ambiguity work outside the current v1 scope.

## Change discipline

- Keep Android framework types at the collection edge. Estimator inputs should be
  plain Kotlin data classes so the core remains straightforward to port to C++/NDK.
- Keep replay deterministic: fixed input ordering, fixed random seeds, and no wall
  clock in estimator results except measured latency.
- Prefer small interfaces between collection, preprocessing, estimation, and UI.
- Do not couple the live estimator to the NDJSON writer. Both should consume the
  same typed sample stream when live processing is introduced.
- Schema changes are additive within version 1. Breaking changes require a new
  `schema_version`, parser support, validation updates, and migration notes.
- Preserve the current dual-pipeline intent: low-latency INS/EKF is authoritative;
  fixed-lag FGO is delayed and health-gated.
- Do not claim Samsung S23+ or sub-meter performance from emulator, synthetic, or
  five-epoch fixture results.

## Required verification

Run the narrowest relevant tests while developing, then run the complete gate:

```bash
/Users/leeboonyao/.gradle/wrapper/dists/gradle-8.13-bin/5xuhj0ry160q40clulazy9h7d/gradle-8.13/bin/gradle \
  -p . :app:testDebugUnitTest :app:assembleDebug :app:lintDebug --offline
```

For log changes, validate at least one stopped/exported session:

```bash
python3 ../validation_tools/validate_ndjson.py /path/to/navgraph_session.ndjson
```

For public replay changes, run the desktop benchmark and compare `summary.json`
with the on-device regression. See `docs/testing_plan.md` for the full matrix.

For every new feature, install the built APK on an Android Studio AVD and run
public smartphone replay through the app UI. Verify the changed behavior during
playback and compare results with the same desktop dataset/configuration. Check
UI sizing on both an S23+-representative and a smaller phone viewport at default
and enlarged font scales. Follow the layout and playback gates in
`docs/testing_plan.md` and record evidence in `docs/emulator_verification.md`.
Unexecuted or blocked checks must be reported as incomplete, not passed.

Add tests when changing:

- coordinate or attitude conventions;
- timestamp alignment or discontinuity behavior;
- pseudorange, Doppler, Earth-rotation, or correction signs;
- residuals, Jacobians, covariance updates, robust losses, or marginalization;
- preprocessing thresholds or rejection reasons;
- schema-required fields or replay output formats.

## Review checklist

Before considering a change complete, confirm that:

- frame and unit assumptions are explicit;
- timestamps come from the intended time base;
- unavailable Android fields are omitted rather than fabricated;
- failures and rejected observations remain inspectable;
- numerical Jacobian checks still pass;
- tests, debug build, and lint pass;
- new features pass Android Studio AVD playback and UI sizing checks, with
  screenshots and replay results recorded;
- documentation states whether evidence is synthetic, emulator, public replay, or
  physical S23+ field data.
