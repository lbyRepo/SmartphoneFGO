# Interfaces

This document summarizes public interfaces between modules. Detailed headers live under `cpp_core/include/s23/`; Python replay/preprocessing APIs live under `python_tools/`.

## Log Interface

Producer: `android_app`.

Consumer: `python_tools/replay` and `python_tools/gnss`.

Format: NDJSON, documented in `docs/log_schema.md`.

Required record families for a useful field log:

- `session_start`
- `sensor_meta`
- `imu`
- `gnss_clock`
- `gnss_raw`
- `fused_location`
- `session_stop`

## GNSS Preprocessing Interface

Python API:

- `preprocess_log(path, config)`
- `preprocess_records(records, config)`
- `compute_pseudorange_m(clock, measurement)`

Outputs:

- `ObservationEpoch`
- accepted `Observation` entries
- rejected measurements with reason strings

Current limitation: real-phone observations are not yet corrected with satellite ephemeris, satellite clock, atmospheric, Earth-rotation, relativistic, or group-delay models.

## Coordinate Interface

Python:

- `Lla`, `Ecef`, `Enu`
- `lla_to_ecef`
- `ecef_to_lla`
- `ecef_to_enu`
- `enu_to_ecef`

C++:

- `s23::Lla`
- `s23::Ecef`
- `s23::Enu`
- `s23::LlaToEcef`
- `s23::EcefToLla`
- `s23::EcefToEnu`
- `s23::EnuToEcef`

Frame convention: ECEF is WGS-84 Earth-centered Earth-fixed meters. ENU is local tangent frame at the specified LLA origin.

## C++ Estimator Interface

Primary public types:

- `s23::Timestamp`
- `s23::Vec3`
- `s23::Quaternion`
- `s23::NavState`
- `s23::EstimatorStatus`
- `s23::GnssObservation`
- `s23::GnssObservationEpoch`
- `s23::RejectedGnssMeasurement`
- `s23::IEphemerisCorrectionProvider`
- `s23::SatelliteCorrection`
- `s23::IRealtimeEstimator`

Synthetic estimator APIs:

- `s23::MakeSyntheticGnssDataset`
- `s23::SolveWls`
- `s23::SolveFixedLagFgo`
- `s23::PseudorangeResidual`
- `s23::DopplerResidual`
- `s23::NumericJacobian`

Prototype baseline APIs:

- `s23::ImuPropagator`
- `s23::EkfGnssInsBaseline`

Current limitation: these estimator APIs are not yet wired to real phone logs because the ephemeris/correction implementation is missing.

## Replay/Evaluation Interface

Single command:

```bash
python3 python_tools/replay/run_replay.py --dataset synthetic --output-dir <dir>
```

or:

```bash
python3 python_tools/replay/run_replay.py --dataset log --input <log.ndjson> --output-dir <dir>
```

Synthetic outputs:

- `truth.csv`
- `android_fused.csv`
- `estimates_wls.csv`
- `estimates_ekf.csv`
- `estimates_fgo.csv`
- `solver_iterations.csv`
- `metrics_summary.json`
- `trajectory_xy.svg`
- `position_error.svg`

Phone-log outputs:

- `observations.csv`
- `rejected_measurements.csv`
- `android_fused.csv`
- `metrics_summary.json`
