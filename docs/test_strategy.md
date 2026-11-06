# Test Strategy

## Stage 0: Repository And Schema

Purpose: prove module boundaries and log schema are stable enough for independent work.

Checks:

- Android app builds.
- Sample NDJSON fixture parses with `--strict`.
- Docs identify owners and forbidden cross-module responsibilities.
- Interface headers compile once a C++ target exists.

## Stage 1: Android Logger Field Readiness

Purpose: prove the S23+ can produce replayable raw data.

Device tests:

- Static open-sky for at least 10 minutes.
- Walking open-sky for at least 5 minutes.
- Screen on and screen off logging.
- App background/foreground transitions while service remains active.

Required record families:

- `session_start`
- `sensor_meta`
- `imu`
- `gnss_clock`
- `gnss_raw`
- `fused_location`
- `session_stop`

Metrics:

- IMU sample rate and jitter.
- GNSS epoch rate.
- Satellite constellation/frequency coverage.
- ADR availability and reset/cycle-slip flags.
- Fused location availability.
- File growth rate and missing/corrupt JSON lines.

## Stage 2: Replay And Conversion

Purpose: turn Android logs into deterministic estimator input streams.

Checks:

- Parser rejects malformed required fields.
- GNSS preprocessing converts raw clock/measurement records into observation epochs.
- Invalid measurements are rejected with explicit reasons.
- LLA, ECEF, and ENU coordinate transforms pass known-value and round-trip tests.
- Replay order uses monotonic timestamps and preserves GNSS clock-domain fields.
- Python tooling can summarize per-record counts and timing gaps.
- A single replay command writes CSV outputs, metrics, and plots.

## Stage 3: Estimator Baselines

Purpose: future work after the current architecture milestone.

Checks:

- WLS residual sanity with synthetic satellite data.
- WLS convergence from a perturbed ECEF initial condition.
- GNSS-only fixed-lag FGO convergence on synthetic pseudorange data.
- Doppler/range-rate factor constrains velocity and clock drift on synthetic data.
- Numerical Jacobian check for pseudorange residuals.
- INS propagation unit tests with constant-acceleration cases.
- Minimal EKF baseline lifecycle and GNSS position correction tests.
- Synthetic replay compares WLS, EKF, FGO, and Android fused baseline outputs reproducibly.

Future checks:

- EKF covariance and innovation-gating tests.
- FGO fixed-lag graph construction tests with real replay logs.
