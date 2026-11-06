# Testing plan

## Principles

Testing is layered so math defects, Android lifecycle defects, and hardware limits
are not confused. Every result must be labeled as one of:

- deterministic unit or synthetic evidence;
- Android emulator behavior;
- public replay behavior;
- physical Samsung Galaxy S23+ field behavior.

Only the last category can support device capability, power, thermal, or field
accuracy claims. Sub-meter accuracy is a measured outcome, not an acceptance
assumption.

## Continuous verification gate

Run from `fgo_android_app`:

```bash
/Users/leeboonyao/.gradle/wrapper/dists/gradle-8.13-bin/5xuhj0ry160q40clulazy9h7d/gradle-8.13/bin/gradle \
  -p . :app:testDebugUnitTest :app:assembleDebug :app:lintDebug --offline
```

The gate passes only when unit tests, debug APK assembly, and lint all succeed.
Warnings that affect permissions, foreground service behavior, precision, or API
compatibility are treated as failures even if lint's exit code is zero.

Run the portable C++ and macOS host-JNI parity gates separately:

```bash
./tools/run_cpp_tests.sh
/Users/leeboonyao/.gradle/wrapper/dists/gradle-8.13-bin/5xuhj0ry160q40clulazy9h7d/gradle-8.13/bin/gradle \
  -p . :app:nativeHostParityTest --offline
```

The second gate compares Kotlin and C++ WLS, nominal INS, complete EKF
state/covariance, and fixed-lag keyframe/marginal output on the same deterministic
inputs. Android packaging and
loadability are covered separately by
`NativeParityInstrumentation`, which must report `native_library=loaded`,
`live_wls_engine=C++ JNI`, `live_ins_engine=C++ JNI`, `live_ekf_engine=C++ JNI`,
2,000 native EKF/INS steps, 21 native GNSS corrections, `live_fgo_engine=C++ JNI`,
at least one native graph window, zero native failures, and accepted FGO feedback.

With an arm64 emulator or device connected, run:

```bash
/Users/leeboonyao/.gradle/wrapper/dists/gradle-8.13-bin/5xuhj0ry160q40clulazy9h7d/gradle-8.13/bin/gradle \
  -p . :app:assembleDebug :app:assembleDebugAndroidTest --offline
/Users/leeboonyao/Library/Android/sdk/platform-tools/adb install -r \
  app/build/outputs/apk/debug/app-debug.apk
/Users/leeboonyao/Library/Android/sdk/platform-tools/adb install -r \
  app/build/outputs/apk/androidTest/debug/app-debug-androidTest.apk
/Users/leeboonyao/Library/Android/sdk/platform-tools/adb shell am instrument -w -r \
  com.example.navgraph.test/com.example.navgraph.estimator.NativeParityInstrumentation
```

## Unit tests

### Coordinates and frames

Required coverage:

- LLA-ECEF-LLA round trips at Singapore, equator, high latitude, negative altitude,
  and near-pole locations;
- ECEF-ENU-ECEF position and vector round trips;
- ENU basis orientation at known references;
- quaternion identity, inverse, composition, normalization, and 90-degree rotation;
- range and range-rate derivative mapping from ECEF to ENU;
- invalid array sizes and non-finite inputs.

Tolerance should reflect double-precision computation and physical units. Do not
use a loose meter-level threshold to hide a sign or axis error.

### Time and mechanization

Required coverage:

- stationary specific force cancels gravity;
- constant acceleration and constant angular rate have analytic outcomes;
- bias compensation and receiver clock propagation;
- midpoint integration over unequal sample spacing;
- duplicate, reversed, and large-gap IMU timestamps;
- GNSS/INS alignment at, inside, and outside the 1 ms boundary;
- a clock discontinuity forces a reset/segment in the live coordinator.

### GNSS preprocessing

Table-driven tests must cover each stable rejection reason independently:

- unsupported constellation or missing time;
- invalid tracking state;
- unsupported/missing frequency;
- low C/N0 and low elevation;
- missing pseudorange inputs;
- invalid pseudorange magnitude or uncertainty;
- invalid/unhealthy/stale satellite state;
- valid pseudorange with invalid Doppler;
- fewer than four accepted satellites in an epoch.

Also test constellation time rollover, correction signs, satellite clock sign,
Earth rotation, frequency classification, uncertainty floors, and deterministic
input ordering.

### Estimator math

Required coverage:

- WLS convergence from cold and warm starts with varied geometry;
- rank-deficient geometry and fewer than four measurements fail cleanly;
- Doppler velocity and clock drift with and without valid rates;
- normalized innovation gates accept inliers and reject injected outliers;
- covariance remains finite, symmetric, and non-negative on its diagonal;
- Joseph update behavior under repeated measurements;
- fixed-lag window boundaries and missing IMU interval handling;
- graph convergence, damping failure, and solver failure reporting;
- central-difference checks for every pseudorange, Doppler, motion, clock, and bias
  Jacobian.

Numerical Jacobian maximum absolute error is currently required below `2e-5` for
pseudorange and Doppler factors.

## Existing deterministic regressions

The current JVM suite checks:

- Singapore WGS84 round trip and ENU axes;
- quaternion rotation, gravity cancellation, and gyroscope propagation;
- the bundled GSDC-derived fixture's WLS/FGO convergence and Jacobians;
- a 20-second, 100 Hz IMU plus 1 Hz GNSS synthetic run with inertial bias, noise,
  and one pseudorange outlier.
- bounded live-queue ordering, draining, overflow telemetry, and post-close behavior;
- Android clock/TOW pseudorange construction into a solver-ready epoch;
- unavailable ephemeris and independent state, signal, frequency, and uncertainty
  rejection paths;
- deterministic satellite snapshot propagation and stale-state rejection.
- Android padded 30-bit GPS L1 C/A word decoding with signed/scaled fields;
- parity, embedded-subframe ID, IODC/IODE, stale-state, health, and signal gates;
- fixed ECEF position, velocity, satellite clock, source-age, and URA regressions
  for an assembled three-subframe broadcast fixture;
- strict Base64 and 40-byte payload validation for logged GPS navigation messages.
- cross-language GPS broadcast-state parity, causal NDJSON event ordering, exact
  callback grouping, future-ephemeris exclusion, clock-arc partitioning, missing
  full-bias rejection, and parseable empty-solver artifacts.
- a 20-second live-backend dispatch with intentionally unrelated GPS and monotonic
  clocks, 2,001 paired IMU samples, 21 WLS/EKF updates, at least 2,000 propagation
  steps, zero stale updates, asynchronous FGO output, skew accounting, and reset.
- dependency-free C++ pseudorange/Doppler WLS convergence, failure handling, and
  numerical Jacobians, plus Kotlin/JNI parity for state, clock, iteration, and
  rejection outputs on a deterministic eight-satellite epoch.
- dependency-free C++ quaternion and nominal INS gravity, acceleration, yaw, bias,
  clock, timestamp, and non-finite-input tests plus full-state Kotlin/JNI parity.
- dependency-free C++ 17-state covariance propagation, ENU/ECEF GNSS derivative
  mapping, sequential pseudorange/Doppler Joseph updates, outlier gates, timestamp
  and non-finite failures, covariance symmetry, and full 289-entry Kotlin/JNI parity.
- dependency-free C++ fixed-lag Gauss-Newton convergence, robust GNSS outlier
  handling, preintegration/frame failures, error reduction, positive selected
  marginals, and Kotlin/JNI parity for every 11-state keyframe and eight marginal
  sigmas.
- fixed-lag marginal covariance extraction, bounded recent FGO-feedback acceptance,
  stable stale/missing-covariance/absolute-bound rejection reasons, and proof that
  correlated feedback does not reduce EKF covariance.

The synthetic test must continue to report 2,001 IMU samples, 21 GNSS epochs, 20
inertial factors, zero solver failures, at least one EKF pseudorange rejection,
finite positive position sigma, and better EKF median horizontal error than WLS.
Exact latency is not a unit-test assertion.

## Log schema tests

For every logger/schema change:

1. Start a session, allow all four streams to produce records, and stop it normally.
2. Parse every line as strict JSON.
3. Run:

```bash
python3 ../validation_tools/validate_ndjson.py /path/to/navgraph_<session>.ndjson
```

4. Assert one `session_start`, one `session_end`, stable session ID, schema version 1,
   and nonzero IMU, clock, raw GNSS, and fused-location counts.
5. Assert record-specific source time equals the common event time.
6. Assert callback time never precedes source event time.
7. Measure rate, jitter, callback delay, and cross-stream offset distributions.
8. Verify an interrupted session remains parseable and is reported as truncated.

Then run the deterministic preprocessing/estimator replay:

```bash
python3 ../validation_tools/navgraph_replay.py /path/to/navgraph_<session>.ndjson \
  --output ../validation_results/<session>
```

Assert that `summary.json` accounts for every raw callback, reports each clock arc
and stable rejection reason, and never uses navigation messages or fused fixes
from a later monotonic timestamp. `solver_status=not_run_no_clean_epochs` is the
required honest outcome when the RF/navigation inputs cannot form a clean epoch.

Add golden parser tests for optional-field omission, unknown additive fields,
malformed JSON, mixed sessions, unsupported versions, and event-time regressions.

## Public replay

Run the desktop reference from the workspace root:

```bash
python3 validation_tools/gsdc_benchmark.py \
  --device-gnss datasets/gsdc_2023_sample/device_gnss.csv \
  --ground-truth datasets/gsdc_2023_sample/ground_truth.csv \
  --output validation_results/gsdc_2023_sample
```

Expected artifacts are `solutions.csv`, `summary.json`, and `trajectory.svg`.
Compare epoch count, accepted/rejected observations, convergence, trajectory,
metrics, and Jacobian errors with Android replay. Floating-point comparisons use
explicit tolerances; timing is reported but not expected to match.

Before accuracy conclusions, add full GSDC traces spanning open sky, urban canyon,
walking, and vehicle motion. Preserve dataset provenance and license information.
Do not tune thresholds against test ground truth.

## Android emulator tests

Use the configured API 36 emulator for lifecycle and rendering, not RF accuracy:

- fresh install and location/notification permission grant and denial;
- start, duplicate start, stop, notification stop, activity recreation, and service
  destruction;
- Live -> Replay -> Diagnostics transitions and replay retry;
- foreground service notification presence;
- log path, closure, and schema validation;
- background replay responsiveness and no main-thread work;
- no `AndroidRuntime` exception, ANR, overlap, or stale logging status;
- portrait layout at 1080 by 2400 and at least one smaller supported viewport.

Record APK version, emulator image/API, viewport, cold/warm replay latency, and
screenshots in `docs/emulator_verification.md`.

### Required in-app playback gate

For every new feature, run the built APK on an Android Virtual Device managed by
Android Studio Device Manager. Automated ADB/instrumentation execution on that
AVD is acceptable; desktop-only replay does not satisfy this gate.

1. Record the APK version, build identifier or SHA-256, AVD name, API level, ABI,
   pixel resolution, density, logical viewport in dp, and system font scale.
2. Load a versioned public smartphone dataset through the app's supported replay
   path. Record source, license, checksum, time interval, estimator settings, and
   any conversion steps. Replay recorded measurements through the Android
   pipeline; emulator route/location injection alone is insufficient.
3. Exercise replay from selection/loading through progress and completion. Check
   available retry and navigation controls, the feature under test, results, and
   diagnostics. Verify responsiveness and absence of crashes, ANRs, or stuck
   loading states. Capture empty/error states where supported.
4. Compare Android outputs with desktop replay for the same input/configuration:
   epoch coverage, estimator status, accepted/rejected measurements, and numerical
   outputs using documented tolerances. Record the actual backend used, including
   native availability or fallback. A reference implementation alone does not
   verify changed Android/native code.
5. Record positional-error metrics against independent ground truth when present,
   and compare with the raw-GNSS WLS baseline. Report qualifying epochs with at
   least 11 distinct usable satellites separately; multiple signals from one
   satellite count once. Record geometry and signal quality as well as count.
   Without truth, label comparisons as disagreement, not accuracy. An insufficient
   qualifying subset must be reported as incomplete accuracy validation.
6. Save representative screenshots and the replay summary, with evidence paths
   and pass/fail results in `docs/emulator_verification.md`. Identify failures and
   blocked checks explicitly. Emulator playback validates software behavior on
   recorded inputs; it does not establish physical S23+ RF or field performance.

### UI sizing acceptance

Use these repository acceptance criteria for the playback gate:

- Test an S23+-representative 1080 x 2340 pixel portrait AVD with recorded density,
  plus a smaller phone viewport of 360 dp width. Pixel dimensions alone do not
  establish equivalent UI size; record actual logical dimensions and insets.
- Test both configurations at system font scales 1.0 and 1.3, and the smaller
  viewport at 2.0. Use dp for layout and sp for text; do not shrink text to defeat
  system font scaling. Use wrapping, reflow, and scrolling for longer content.
- Require interactive touch targets of at least 48 x 48 dp. Keep primary data and
  body text at least 14 sp at the default scale, with clearly readable labels and
  units. Verify actual rendered size and touch bounds, not just source constants.
- No unintended overlap, clipped text, offscreen controls, or content obscured by
  status/navigation bars, cutouts, or the keyboard. Essential values, error
  messages, and units must remain readable; scrolling must reach all content.
- Inspect Live, Replay, and Diagnostics, including loading, active playback,
  completion, and supported error states. Check long filenames, large counters,
  missing values, plots, legends, buttons, and tabs without layout shifts.
- Exercise rotation if supported. If portrait is intentionally locked, document
  that constraint. Ensure background/foreground transitions preserve usable UI.
- Capture and visually inspect screenshots at each viewport/font configuration;
  use UI hierarchy or instrumentation checks for bounds and touch targets. A
  screenshot alone does not demonstrate functional controls or correct replay.
- Fix layout failures and repeat affected checks before marking feature validation
  complete. Record any unavailable emulator, fixture, or unsupported test state.

## Samsung Galaxy S23+ capability tests

Run outdoors with a foreground service and screen state documented. For each
session capture device/firmware fingerprint and sensor metadata.

Go/no-go checks before live estimation:

- raw GNSS callbacks and complete clock fields are present;
- at least four usable pseudoranges are repeatedly derivable;
- pseudorange rate and uncertainty are finite;
- accelerometer and gyroscope achieve at least 100 Hz without sustained gaps;
- event timestamps are monotonic and callback latency is bounded;
- logs export and replay deterministically.

Measure rather than assume:

- constellation and signal/frequency availability;
- actual GNSS epoch rate and full-tracking effect;
- ADR validity, reset/cycle-slip rate, and continuity by signal;
- IMU rate, jitter, duplicate timestamps, FIFO/batching, and dropped samples;
- GNSS-to-IMU offset stability;
- power draw, battery drain, temperature, throttling, memory, and CPU.

ADR/TDCP work has a separate gate: long valid arcs, understood discontinuities,
repeatable cycle-slip detection, and improvement over pseudorange/Doppler baselines.

## Field scenarios

Collect repeated, independently replayable runs for:

- static open sky;
- walking and vehicle motion in open sky;
- Singapore urban canyon;
- tree canopy;
- indoor/outdoor transition;
- controlled GNSS outage and reacquisition;
- hand, pocket, bag, and rigid vehicle mounting;
- low battery and thermal stress.

Ground truth should come from a surveyed point, geodetic receiver, or accurately
time-aligned reference trajectory. Android fused location is a comparison baseline,
not ground truth.

## Metrics and acceptance

Report, by scenario and estimator:

- horizontal/vertical error, CEP50/CEP95, and maximum error when truth exists;
- velocity and heading error when reference data exists;
- trajectory second-difference smoothness and position jumps;
- output/update rate, end-to-end latency, and solution age;
- accepted/rejected measurement counts and reason distribution;
- solver iterations, failures, divergence, and recovery time;
- covariance consistency and innovation statistics;
- CPU, memory, battery drain, temperature, and throttling.

The Android logger milestone passes when a stopped S23+ log contains all required
streams and passes strict validation/replay. The preprocessing milestone passes
when invalid observations have explicit reasons and clean epochs are reproducible.
Estimator integration passes when WLS, EKF, and FGO run on identical epochs without
hidden future data and publish inspectable health. A sub-meter claim requires
repeatable physical-device field results against independent ground truth and
must state scenario, percentile, latency, and failure rate.

## Current gaps

The following remain required before the full target can be accepted:

- multi-constellation/modernized-signal ephemeris;
- attitude and gyroscope-bias variables in the fixed-lag graph;
- physical S23+ rate, latency, power, thermal, and field-accuracy profiling;
- additional public traces containing IMU/Doppler and independently referenced S23+
  field campaigns.
