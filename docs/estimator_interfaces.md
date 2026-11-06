# Estimator Interface Definitions

The C++ estimator core is Android-independent. Android and Python replay code should feed it typed measurement records rather than Android framework objects or raw JSON.

Header contracts live under `cpp_core/include/s23/`.

## Core Types

- `s23::Timestamp`: monotonic nanoseconds plus optional wall-clock correlation.
- `s23::Vec3`: small fixed-size vector for interface values.
- `s23::Quaternion`: attitude representation.
- `s23::NavState`: position, velocity, attitude, IMU biases, receiver clock bias/drift.
- `s23::EstimatorStatus`: health flags, accepted/rejected measurement counts, latency, and status message.

## Measurement Inputs

- `s23::ImuSample`: accelerometer or gyroscope sample from Android sensor timestamps.
- `s23::GnssClockSample`: one Android GNSS clock record per raw measurement callback.
- `s23::GnssRawMeasurement`: one Android raw GNSS satellite measurement.
- `s23::GnssEpoch`: clock plus all satellite measurements for one callback epoch.
- `s23::GnssObservation`: preprocessed pseudorange and pseudorange-rate observation after QC.
- `s23::GnssObservationEpoch`: accepted observations plus explicit rejected-measurement reasons.
- `s23::FusedLocationSample`: Android fused baseline for evaluation only.

## GNSS Correction Interface

- `s23::IEphemerisCorrectionProvider`: supplies satellite position, velocity, clock, atmospheric, relativistic, and group-delay corrections for a preprocessed observation.
- `s23::SatelliteCorrection`: correction output used by future WLS, EKF, and FGO residual builders.

The preprocessing layer prepares observations; future residual builders apply ephemeris/correction models.

## Estimator Interfaces

- `s23::IRealtimeEstimator`: live estimator API for future WLS/EKF/FGO hybrid work.
- `Initialize(config)`: sets rates, window length, and feature flags.
- `PushImu(sample)`: accepts IMU samples in monotonic timestamp order.
- `PushGnssEpoch(epoch)`: accepts one raw GNSS epoch.
- `PushFusedLocation(sample)`: optional baseline/evaluation channel; not used as truth.
- `LatestState()`: returns the latest live navigation state.
- `LatestStatus()`: returns health and diagnostics.
- `Reset(reason)`: clears estimator state after explicit lifecycle or divergence events.

## Versioning Rules

- Log schema changes must remain backward compatible within `schema_version = 1`, or bump the version and update the parser.
- Interface changes in `cpp_core/include/s23/` must be reflected in this document.
- Android code must not call concrete estimator classes directly; future JNI should depend on the interface layer.
