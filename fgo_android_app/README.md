# NavGraph Android prototype

`fgo_android_app` is the emulator-validated Android front end for the GNSS/INS/FGO
prototype. It is intentionally standalone because the legacy `android_app` source
tree could not be read through the current macOS workspace grant.

Implemented:

- Runtime location and notification permissions.
- Foreground start/stop recording service.
- Raw GNSS measurements and GNSS clock records with separate event and callback
  monotonic timestamps.
- Raw `GnssNavigationMessage` logging plus parity-checked GPS L1 C/A subframe
  assembly, broadcast-orbit/clock evaluation, URA weighting, and live provider
  health diagnostics.
- Accelerometer and gyroscope recording at a requested 10,000 microsecond period.
- Fused, GPS, and network location baselines without writing stale last-known fixes.
- Live, replay, and diagnostics views.
- On-device schema-aware GSDC preprocessing for rich 2023 and legacy 2021 derived
  files, with unavailable source fields preserved as unavailable.
- On-device robust pseudorange WLS and two-second fixed-lag pseudorange/Doppler FGO.
- Quaternion body-to-ENU IMU mechanization with accelerometer/gyro bias and receiver
  clock states.
- A 17-state-error GNSS/INS EKF with Joseph covariance updates, pseudorange/Doppler
  innovation gates, and sparse 100 Hz covariance propagation.
- A deterministic 100 Hz IMU plus 1 Hz GNSS benchmark with sensor bias, measurement
  noise, and an injected pseudorange outlier.
- A two-second inertial fixed-lag graph with preintegrated position/velocity,
  accelerometer-bias random walk, pseudorange, Doppler, and clock factors.
- A live Kotlin backend with warm-start WLS, gravity-aligned 100 Hz IMU/EKF
  propagation, exact monotonic GNSS/IMU interval splitting, clock-arc reset, and
  asynchronous two-second FGO publication.
- Covariance-, age-, convergence-, innovation-, and magnitude-gated delayed FGO
  feedback into the EKF, using conservative partial corrections without false
  covariance reduction for correlated measurements.
- Numerical factor Jacobian checks, solver health, latency metrics, and local ENU
  truth/WLS/FGO trajectory rendering.
- A provenance-tracked 2,081-epoch Samsung S20 Ultra public benchmark plus a
  600-second, 15,554-observation segment executed inside the APK.
- A dependency-free C++17 GNSS geometry, robust pseudorange/Doppler WLS, quaternion,
  full-state midpoint INS mechanization, 17-state error EKF, and dense fixed-lag
  graph core with deterministic Kotlin/JNI parity.
- Default arm64 JNI WLS and complete 100 Hz EKF propagation/GNSS-correction paths
  with inspectable Kotlin fallbacks. Dependency-free Android instrumentation
  exercises the packaged library through the complete synthetic live backend.

Not yet implemented in this module:

- Galileo, BeiDou, GLONASS, QZSS, and modernized GPS navigation-message decoders;
  the live broadcast provider currently serves GPS L1 C/A only.
- Attitude and gyro-bias variables in the fixed-lag graph. The current inertial graph
  uses translational preintegration; quaternion and gyro-bias estimation are in the
  EKF.
- Physical S23+ rate, latency, power, thermal, and field-accuracy profiling. WLS,
  nominal INS, EKF covariance propagation/GNSS correction, and fixed-lag graph
  optimization are native by default with Kotlin reference fallbacks.

Build:

```bash
/Users/leeboonyao/.gradle/wrapper/dists/gradle-8.13-bin/5xuhj0ry160q40clulazy9h7d/gradle-8.13/bin/gradle \
  -p fgo_android_app :app:testDebugUnitTest :app:assembleDebug --offline
```

APK: `fgo_android_app/app/build/outputs/apk/debug/app-debug.apk`

Portable C++ and host JNI parity can be verified without the Android NDK:

```bash
cd fgo_android_app
./tools/run_cpp_tests.sh
/Users/leeboonyao/.gradle/wrapper/dists/gradle-8.13-bin/5xuhj0ry160q40clulazy9h7d/gradle-8.13/bin/gradle \
  -p . :app:nativeHostParityTest --offline
```

The default build pins NDK `29.0.14206865`, SDK CMake `3.22.1`, and `arm64-v8a`.
Use `-Pnavgraph.enableNative=false` only for a deliberate Kotlin-only fallback
build on a machine without that toolchain.

The public regression fixtures are bundled under `app/src/main/assets`. See
`docs/emulator_verification.md` for the verified runtime path, measured emulator
latencies, limitations, and screenshots. `docs/estimator_design.md` records the
frame, timestamp, state, update, and factor contracts. The sustained real-data
results and provenance are in `docs/validation_report.md`.

Exported schema-v1 logs can be replayed from the workspace root with:

```bash
python3 validation_tools/navgraph_replay.py device_logs/<session>.ndjson \
  --output validation_results/<session>
```

The replay causally reconstructs clock arcs, GPS L1 C/A ephemerides, corrected
pseudorange/range-rate epochs, IMU and fused streams, then runs WLS/FGO when the
log contains at least four solver-ready observations per epoch.
