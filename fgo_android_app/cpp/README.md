# Portable estimator core

This directory contains dependency-free C++17 estimator code intended for both
desktop regression and Android NDK/JNI integration. The public ABI in
`include/navgraph/gnss.hpp` mirrors the Kotlin solver-ready observation contract:
satellite states and corrections are already prepared upstream, ECEF is explicit,
and residual signs are predicted minus observed. `include/navgraph/ins.hpp` defines
the ENU nominal INS state, body-to-ENU quaternion, body-frame IMU samples, biases,
receiver clock, and typed propagation statuses.
`include/navgraph/ekf.hpp` defines the row-major 17 by 17 covariance, explicit
ECEF-to-ENU frame, Joseph-form pseudorange/Doppler update, health deltas, and typed
failure statuses.
`include/navgraph/fgo.hpp` defines the 11-state fixed-lag keyframe, translational
preintegration input, robust GNSS graph window, selected marginal sigmas, and typed
solver status. `include/navgraph/frame.hpp` is the shared local-frame implementation
used by both filters.

Run the current host parity gate from `fgo_android_app`:

```bash
./tools/run_cpp_tests.sh
```

The tests cover pseudorange WLS, Doppler velocity/clock drift, numerical
Jacobians, rank-deficient geometry, non-finite weights, gravity cancellation,
constant acceleration/yaw, bias cancellation, clock propagation, and timestamp
failures. Android JNI wiring stays separate from the portable APIs.
EKF tests additionally cover sparse covariance propagation, sequential GNSS
correction, innovation outlier gates, non-finite observations, local-frame mapping,
and covariance symmetry.
FGO tests cover damped Gauss-Newton convergence, Huber outlier handling, local-frame
and preintegration failures, error reduction, and positive marginal uncertainty.

Run host JNI-to-Kotlin parity with:

```bash
gradle -p fgo_android_app :app:nativeHostParityTest --offline
```

This compiles `cpp/jni/navgraph_jni.cpp` against the installed JDK headers, loads
the resulting native library into the JVM test process, and compares Kotlin and
C++ WLS, INS, complete EKF state/covariance, and fixed-lag keyframe/marginal outputs
on deterministic inputs.

NDK `29.0.14206865` and SDK CMake `3.22.1` are pinned. Build the arm64 Android
library and package it into the APK with the ordinary build:

```bash
gradle -p fgo_android_app :app:assembleDebug
```

Use `-Pnavgraph.enableNative=false` for an explicit Kotlin-only APK. With a healthy
packaged JNI library, live WLS, EKF, and FGO select `C++ JNI`; a load or numeric
failure falls back to Kotlin and increments an inspectable subsystem-specific health
counter.
