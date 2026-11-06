# Estimator design

## Frame contract

- GNSS satellite and receiver positions are ECEF meters; velocities are ECEF m/s.
- The inertial nominal state uses a local ENU frame whose origin is an initial ECEF
  receiver position.
- `attitudeBodyToEnu` rotates Android sensor-body vectors into ENU. Accelerometer
  values are treated as specific force. ENU gravity is `[0, 0, -9.80665]` m/s2.
- GNSS range and range-rate derivatives are formed in ECEF and explicitly mapped
  into ENU before EKF or graph use.

## Time contract

- IMU ordering and propagation use monotonic nanoseconds. Android collection uses
  `SensorEvent.timestamp`, which shares the elapsed-realtime time base.
- GNSS events use `GnssClock.elapsedRealtimeNanos` when available and retain the
  callback timestamp separately.
- Wall-clock milliseconds are metadata and are never used for inertial integration.
- A GNSS update requires its epoch and propagated INS state to agree within 1 ms.
  Non-increasing IMU samples are counted and not propagated.

## EKF contract

The nominal state contains local position, local velocity, body-to-ENU quaternion,
accelerometer bias, gyro bias, receiver clock bias, and receiver clock drift. The
17-dimensional error state is:

`[position(3), velocity(3), attitude_error(3), accel_bias(3), gyro_bias(3), clock_bias, clock_drift]`

Propagation uses midpoint angular rate and specific force. Covariance propagation
uses the same attitude and bias linearization with sparse process couplings. GNSS
updates are sequential scalar pseudorange and Doppler updates with 5-sigma normalized
innovation gates and Joseph-form covariance updates.

## FGO contract

Each synthetic fixed-lag keyframe currently contains ENU position, ENU velocity,
accelerometer bias, clock bias, and clock drift. Factors include:

- Robust pseudorange and Doppler measurements.
- IMU-preintegrated position and velocity over each one-second keyframe interval.
- Accelerometer-bias random walk.
- Clock bias/drift evolution.
- A prior on the first state in each two-second window.

The preintegrator also computes delta attitude, but attitude and gyro bias are not
yet graph variables. This is a known implementation boundary, not a claim of full
visual-inertial-style preintegration.

The Kotlin reference and portable C++17 backend implement the same damped dense
Gauss-Newton window, Huber GNSS factors, state ordering, convergence threshold, and
selected inverse-Hessian marginals. Live Android prefers C++ and records every native
window/fallback; the two-second lag bounds the expected 1 Hz live matrix to 33 by 33.

## Validation boundary

The GSDC fixture validates real public GNSS parsing and GNSS-only WLS/FGO behavior.
It contains no IMU records. The deterministic scenario validates mechanization,
bias handling, EKF updates, inertial graph factors, outlier rejection, and numerical
Jacobians. Neither replaces full public traces or Samsung S23+ field logs.
