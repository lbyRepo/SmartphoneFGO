# Architecture

This repository is a staged Samsung Galaxy S23+ GNSS/INS/FGO prototype. The current goal is to make data collection, replay, preprocessing, and synthetic estimator experiments reproducible before claiming real-phone positioning performance.

## Top-Level Modules

- `android_app/`: Android Kotlin app for runtime permissions, foreground service logging, raw GNSS measurements, GNSS clocks, IMU samples, and Android fused-location baseline records.
- `python_tools/`: deterministic replay, NDJSON validation, GNSS preprocessing, synthetic estimator comparison, metrics, and SVG plots.
- `cpp_core/`: portable C++ types and first synthetic estimator core. This layer must remain Android-NDK friendly and free of Android framework dependencies.
- `tests/`: schema fixtures, Python unit tests, C++ compile/run tests, and synthetic replay tests.
- `docs/`: architecture, schema, interface, testing, roadmap, and review notes.

## Data Flow

```text
Samsung S23+ sensors
  -> android_app logger
  -> NDJSON log file
  -> python_tools/replay parser
  -> python_tools/gnss preprocessing
  -> observation epochs / rejected measurement reports
  -> future ephemeris/correction provider
  -> WLS / EKF / FGO estimators
  -> CSV estimates, metrics JSON, plots
```

The synthetic path bypasses Android logs and builds deterministic satellite/receiver measurements for estimator development:

```text
synthetic dataset
  -> WLS / EKF baseline / fixed-lag FGO
  -> CSV estimates, solver iterations, metrics, plots
```

## Current Estimator Scope

Implemented:

- Synthetic GNSS data generation.
- WLS pseudorange baseline on synthetic data.
- GNSS-only fixed-lag Gauss-Newton FGO on synthetic data.
- Doppler/range-rate factor in synthetic FGO.
- Minimal IMU propagation and EKF baseline shell.

Not implemented:

- Broadcast/precise ephemeris parsing.
- Satellite clock, Earth rotation, relativistic, ionosphere, troposphere, group-delay, and inter-signal bias corrections.
- Real-phone WLS/EKF/FGO positioning from logs.
- Real strapdown INS mechanization.
- Carrier phase ambiguity resolution.
- TDCP.

## Important Frame And Time Rules

- Logged Android IMU values are device/body-frame measurements, not ECEF/navigation-frame acceleration.
- Fused location is a baseline/evaluation source only; it is not ground truth.
- `wall_time_millis` is for human correlation only.
- GNSS clock-domain fields must be interpreted with constellation-specific time-system rules.
- Event timestamps and callback receipt timestamps must stay distinct. Callback receipt time is useful for latency, not measurement time.

## Known Architectural Risks

- The current phone-log path stops after preprocessing because no ephemeris/correction provider exists yet.
- Current synthetic FGO tests use strong synthetic priors and should not be interpreted as field performance evidence.
- The current EKF baseline is not a full covariance-bearing GNSS/INS EKF.
