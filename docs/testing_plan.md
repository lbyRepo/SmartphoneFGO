# Testing Plan

Testing is staged so synthetic estimator work does not masquerade as real S23+ positioning validation.

## Required Local Checks

Android build:

```bash
/Users/leeboonyao/.gradle/wrapper/dists/gradle-8.13-bin/5xuhj0ry160q40clulazy9h7d/gradle-8.13/bin/gradle :android_app:app:assembleDebug --offline
```

Python tests:

```bash
python3 -m unittest discover -s tests/python -v
```

C++ estimator smoke test:

```bash
clang++ -std=c++17 -Icpp_core/include tests/cpp/estimator_core_test.cpp -o /private/tmp/s23_estimator_core_test
/private/tmp/s23_estimator_core_test
```

Replay command:

```bash
python3 python_tools/replay/run_replay.py --dataset synthetic --output-dir /private/tmp/s23_replay_synthetic
```

## Unit Test Coverage

Current coverage:

- NDJSON schema validation.
- GNSS preprocessing acceptance/rejection reasons.
- LLA/ECEF/ENU coordinate transforms.
- Synthetic WLS convergence.
- Synthetic GNSS-only FGO convergence.
- Pseudorange numerical Jacobian check.
- Minimal IMU/EKF baseline lifecycle.
- Replay artifact generation and metrics summary.

Required next tests:

- GNSS clock conversion with realistic Android `full_bias_nanos`, `bias_nanos`, week rollover, negative propagation time, and missing fields.
- Constellation-specific pseudorange time handling for GPS, QZSS, Galileo, BeiDou, and GLONASS before enabling those constellations for real positioning.
- GNSS epoch grouping using `GnssClock.elapsedRealtimeNanos` where available.
- Doppler factor numerical Jacobian including position partial derivatives.
- WLS/FGO behavior under poor geometry, fewer than four satellites, outliers, high uncertainty, and singular normal equations.
- Real S23+ static-open-sky log replay after ephemeris support exists.

## Field Test Plan

Stage 1: logger capability.

- Static open-sky S23+ log, at least 10 minutes.
- Walking open-sky log, at least 5 minutes.
- Screen-on and screen-off logging.
- Confirm raw GNSS, GNSS clock, IMU, and fused-location records.

Stage 2: preprocessing.

- Verify epoch counts, satellite counts, C/N0 distribution, frequency distribution, ADR state distribution, and rejection reasons.
- Do not claim position accuracy at this stage.

Stage 3: real positioning.

- Add ephemeris/correction provider.
- Run WLS on static open-sky logs.
- Compare against surveyed or high-quality reference truth.
- Only then compare EKF/FGO against WLS and Android fused location.

## Merge Gates

- No generated `__pycache__`, `tests/evaluation_outputs`, APKs, or pulled device logs committed.
- Any schema change updates `docs/log_schema.md`, parser validation, and fixtures.
- Any public C++ interface change updates `docs/interfaces.md` and compile tests.
- Any claim about real accuracy must cite a real log, reference truth source, and metrics output.
