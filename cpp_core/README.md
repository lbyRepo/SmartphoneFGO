# C++ Core

This module contains Android-independent estimator interfaces and future implementation boundaries.

Current state: preprocessing plus first synthetic estimator core. The implemented estimator path is intentionally simple: synthetic GNSS, WLS pseudorange baseline, GNSS-only fixed-lag Gauss-Newton FGO with pseudorange and Doppler/range-rate factors, IMU propagation interface, and a minimal EKF baseline shell.

Planned layout:

- `include/s23/core/`: time, vector, quaternion, state, and status types.
- `include/s23/coordinates/`: WGS-84 LLA/ECEF/ENU coordinate transforms.
- `include/s23/gnss/`: raw GNSS clock, clean observation, and ephemeris/correction interfaces.
- `include/s23/math/`: small dense linear algebra utilities.
- `include/s23/estimator/`: synthetic data, WLS, fixed-lag FGO, diagnostics, IMU propagation, and EKF baseline shell.
- `include/s23/ins/`: IMU sample interfaces.
- `include/s23/estimator/`: live estimator API.
- `src/`: future implementations.
- `tests/`: future compile/unit tests.

## Local Checks

```bash
clang++ -std=c++17 -Icpp_core/include tests/cpp/estimator_core_test.cpp -o /private/tmp/s23_estimator_core_test
/private/tmp/s23_estimator_core_test
```

Inspectable solver run:

```bash
clang++ -std=c++17 -Icpp_core/include tests/cpp/estimator_demo.cpp -o /private/tmp/s23_estimator_demo
/private/tmp/s23_estimator_demo
```
