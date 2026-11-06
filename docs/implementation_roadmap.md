# Implementation Roadmap

## Milestone 0: Architecture Skeleton

Status: current milestone.

Checklist:

- Required top-level folders exist: `android_app/`, `cpp_core/`, `python_tools/`, `tests/`, `docs/`.
- `AGENTS.md` defines edit boundaries and verification expectations.
- `docs/module_ownership.md` maps module responsibilities.
- `docs/log_schema.md` defines the Android GNSS/IMU NDJSON schema.
- `cpp_core/include/s23/` defines estimator-facing interfaces before implementation.
- `docs/test_strategy.md` describes staged verification.
- Android logger builds.
- Sample and exported logs parse with the replay validator.

## Milestone 1: S23+ Device Capability Logging

Deliverables:

- Run the Android logger on the Samsung Galaxy S23+.
- Collect static open-sky and walking logs.
- Produce a capability report for raw GNSS, dual-frequency, ADR, GNSS rate, and IMU rate.
- Add one short anonymized S23+ fixture if licensing/privacy allows.

Go/no-go:

- Raw GNSS, GNSS clock, accelerometer, gyroscope, and fused location records are present and parseable.

## Milestone 2: Replay Normalization

Deliverables:

- Python replay stream model for IMU, GNSS epochs, and baseline location.
- Timing summary reports.
- Schema regression tests.

Go/no-go:

- The same log replays deterministically and produces identical record counts/timing summaries.

## Milestone 3: Baseline Estimator Interfaces To Implementations

Deliverables:

- C++ compile path for estimator headers.
- Synthetic GNSS dataset.
- WLS pseudorange baseline.
- GNSS-only fixed-lag FGO with pseudorange factors.
- Doppler/range-rate factor.
- INS propagation interface.
- Minimal EKF GNSS/INS baseline shell.
- JNI bridge design, still no FGO dependency.

Go/no-go:

- C++ unit tests compile and run on desktop; WLS and FGO converge on synthetic data.

## Milestone 4: EKF And Offline FGO Research Prototype

Deliverables:

- EKF GNSS/INS baseline.
- Offline fixed-lag FGO replay prototype.
- Metrics against Android fused location and WLS.

Go/no-go:

- FGO improves replay smoothing or identifies clear data-quality limits.
