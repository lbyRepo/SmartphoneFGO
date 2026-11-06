# Validation report

## Evidence classes

NavGraph uses four separate evidence classes. They are not interchangeable.

| Evidence | Purpose | Accuracy claim allowed |
|---|---|---|
| GSDC 2021 Samsung S20 Ultra full drive | Sustained real smartphone pseudorange replay | Dataset-specific WLS/FGO metrics only |
| GSDC 2021 Samsung 600-second segment | On-device sustained replay and desktop parity | Same segment only |
| GSDC 2023 Pixel 7 Pro five-second fixture | Rich-schema parser, Doppler, and Jacobian smoke test | None; too short |
| Deterministic GNSS/INS simulation | EKF, IMU, bias, outlier, and inertial-factor regression | Implementation behavior only |

No result in this document is Samsung S23+ field evidence.

## Native estimator parity

The portable estimator slice is a dependency-free C++17 implementation of GNSS
geometry, robust pseudorange/Doppler WLS, quaternion math, nominal INS
mechanization, the complete 17-state EKF step, and the 11-state fixed-lag graph.
A deterministic eight-satellite
host test solves position to `4.1e-9 m` and velocity to `3.2e-13 m/s` in the exact
synthetic fixture; central-difference pseudorange and Doppler Jacobian errors are
below `2.2e-6` and `2.6e-10`. A separate JVM test loads the JNI library and matches
the Kotlin solution within `1e-6 m` position and `1e-7 m/s` velocity, including
clock, iterations, and rejection counts.

The default APK now packages a stripped arm64 library built with NDK
`29.0.14206865` and CMake `3.22.1`. API 36.1 ARM instrumentation loaded it and
reproduced `4.01e-9 m` position and `4.04e-13 m/s` velocity error in four
iterations. The complete synthetic live stream selected `C++ JNI` for all 21 WLS
updates, all 2,000 EKF/INS propagation intervals, 21 EKF GNSS updates, and 44 graph
window solves with zero native failures. It published ten asynchronous FGO results
and accepted ten gated feedback corrections. C++ INS has analytic gravity,
acceleration, yaw, bias, clock, and failure-status tests plus full-state host JNI
parity within `2e-12`; EKF parity covers all 289 covariance entries, and FGO parity
covers every keyframe and selected marginal. This establishes Android ABI
packaging and functional integration, not S23+ runtime or field accuracy.

## NavGraph raw-log replay

Schema-v1 Android exports now have a deterministic desktop path through clock-arc
reconstruction, GPS L1 C/A navigation decoding, raw pseudorange/range-rate
preprocessing, and the same WLS/FGO implementation used below. The runner exports
clean observations, IMU, fused baseline, solutions, summary metrics, and an ENU
trajectory from one command. Android fused separation is labeled as a baseline
comparison, not ground-truth error.

The API 36 emulator verification log contains 14,470 records: 13,798 IMU samples,
68 GNSS clock callbacks, 476 raw measurements, and 124 fused fixes. Replay confirms
one clock arc and 100 Hz accelerometer/gyroscope streams, but produces zero clean
epochs because emulator GNSS timing is nonphysical and the emulator emits no
broadcast navigation messages. The solver therefore reports
`not_run_no_clean_epochs`; this is lifecycle/schema evidence only. Artifacts are
under `../../validation_results/navgraph_070_emulator`.

## Public Samsung source

The complete dynamic trace is collection `2021-04-28-US-SJC-1`, phone
`SamsungS20Ultra`, mirrored by the Apache-2.0 AILocAR `androidGnss` repository at
commit `1f8d4bf12eedc0ba0efe4dbbcf103cba380a8599`:

<https://github.com/AILocAR/androidGnss>

The full local copy has 2,081 GNSS epochs and 53,619 derived observations. A
deterministic middle-route segment with 600 epochs and 15,554 observations is
bundled into the APK. Checksums, source paths, retrieval date, and selection bounds
are stored under `../../datasets/gsdc_2021_s20ultra_sjc` and
`../../datasets/gsdc_2021_s20ultra_segment`.

This mirror provides corrected pseudorange ingredients and satellite ECEF state,
but not pseudorange rate, C/N0, elevation, or IMU. The replay therefore does not
invent Doppler or quality values. It exercises pseudorange WLS and pseudorange plus
motion/clock fixed-lag FGO.

## Full-drive result

Desktop replay uses a two-second fixed lag and matches all 2,081 GNSS epochs to
ground truth within plus or minus 1 ms.

| Metric | WLS | Fixed-lag FGO |
|---|---:|---:|
| Horizontal P50 | 10.148 m | 10.128 m |
| Horizontal P95 | 25.291 m | 24.393 m |
| Horizontal mean | 11.313 m | 11.326 m |
| Maximum horizontal error | 53.908 m | 54.137 m |
| Maximum ECEF position jump | 83.434 m | 54.969 m |
| Second-difference RMS | 13.776 m | 10.296 m |
| Mean solve latency | 0.63 ms | 2.35 ms |
| Epochs not meeting convergence threshold | 83 | 96 |
| Post-fit rejected measurements | 297 | 296 |

The graph improves P95 modestly, maximum jump substantially, and trajectory
smoothness. It does not materially improve central horizontal accuracy, slightly
worsens mean/max error, and has more epochs that do not meet the strict 1 mm update
threshold within the iteration limit. This is a noisy-pseudorange baseline, not a
successful sub-meter solution.

Artifacts:

- `../../validation_results/gsdc_2021_s20ultra_sjc/summary.json`
- `../../validation_results/gsdc_2021_s20ultra_sjc/solutions.csv`
- `../../validation_results/gsdc_2021_s20ultra_sjc/trajectory.svg`

## Android replay segment

The 600-second segment produces matching desktop/Kotlin accuracy and health counts:

| Metric | WLS | Fixed-lag FGO |
|---|---:|---:|
| Horizontal P50 | 8.904 m | 8.991 m |
| Horizontal P95 | 20.433 m | 20.204 m |
| Maximum ECEF position jump | 40.677 m | 37.941 m |
| Second-difference RMS | 11.870 m | 9.160 m |
| Epochs not meeting convergence threshold | 1 | 3 |
| Post-fit rejected measurements | 15 | 14 |

Here the graph makes median error slightly worse while reducing tail error, jumps,
and second differences. The UI displays unavailable Doppler Jacobians as `n/a`.
Measured emulator runtime is recorded separately in `emulator_verification.md`.

## Rich GNSS smoke fixture

The five-epoch GSDC 2023 Pixel 7 Pro fixture remains in the app assets and unit
tests. It exercises the rich derived schema, frequency/state/C/N0/elevation QC,
pseudorange rate, satellite velocity, Doppler factors, and numerical Jacobians. It
is deliberately not the primary UI benchmark because five seconds is too short for
performance validation.

## Synthetic GNSS/INS regression

The deterministic 20-second scenario contains 2,001 IMU samples, 21 GNSS epochs,
sensor bias/noise, eight satellites, and one 38 m pseudorange outlier.

| Horizontal metric | WLS | EKF | Inertial FGO (5 s) |
|---|---:|---:|---:|
| P50 | 2.42 m | 0.83 m | 0.96 m |
| P95 | 3.99 m | 1.35 m | 2.03 m |
| Mean | 2.24 m | 0.79 m | 1.08 m |
| Max position jump | 21.07 m | 2.77 m | 3.64 m |
| Second-difference RMS | 14.78 m | 1.17 m | 2.32 m |

The FGO row now uses the five-second default horizon with the twelve-keyframe cap.
At the previous two-second horizon the same scenario produced 1.08 m P50 and 2.93 m
P95, so the longer horizon removes about 31 percent of FGO P95 error without a
convergence failure. The EKF rejects the injected pseudorange and reports finite
position covariance. These values verify deterministic estimator behavior only.

The same stream also drives `KotlinLiveEstimatorBackend` with absolute GNSS
milliseconds deliberately unrelated to monotonic integration time. The regression
pairs all 2,001 accelerometer/gyro samples, produces 21 warm-start WLS and 21 EKF
corrections, performs at least 2,000 propagation steps with no stale update, and
publishes asynchronous fixed-lag FGO output. The final arm64 run completed 44 native
graph windows without fallback. It waits at the first eligible
keyframe and proves at least one covariance-, age-, convergence-, innovation-, and
magnitude-gated correction is accepted. Later deliberately accelerated graph
outputs may be rejected as stale. This proves live dispatch, clock-domain
separation, conservative feedback, and concurrency behavior on synthetic inputs;
it is not S23+ evidence.

## Reproduction

From the workspace root:

```bash
python3 validation_tools/gsdc_benchmark.py \
  --device-gnss datasets/gsdc_2021_s20ultra_sjc/device_gnss.csv \
  --ground-truth datasets/gsdc_2021_s20ultra_sjc/ground_truth.csv \
  --output validation_results/gsdc_2021_s20ultra_sjc \
  --dataset-name "GSDC 2021 / Samsung S20 Ultra / SJC full drive" \
  --dataset-scope "2,083-second public dynamic drive; derived pseudorange only"
```

The implementation and all thresholds run without ground-truth feedback. Ground
truth is used only after estimation to compute metrics and render trajectories.

Raw NavGraph logs use:

```bash
python3 validation_tools/navgraph_replay.py device_logs/<session>.ndjson \
  --output validation_results/<session>
```
