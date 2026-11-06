# Interfaces

## Design rules

Interfaces separate Android collection, GNSS preprocessing, estimator math,
replay, and presentation. Types crossing the estimator boundary must carry units
and frame semantics. Android framework objects and JSON objects do not cross that
boundary.

The Kotlin types below are implemented unless marked **target**. A concrete
interface can be implemented while its production data source remains a target;
for example, deterministic snapshot ephemeris exists, but broadcast ephemeris
decoding does not.

## Collection boundary

The current collectors write schema-v1 records through:

```kotlin
fun NdjsonLogWriter.write(
    recordType: String,
    payload: JSONObject,
    eventElapsedNanos: Long,
)
```

Collection rules:

- the caller selects the source event timestamp;
- the writer adds schema version, record type, session, and wall-write time;
- calls are synchronized and append one complete JSON line;
- optional unavailable fields are omitted by the collector;
- `close(reason)` appends `session_end` before closing.

Live estimation does not parse JSON. Collectors publish `RawImuSample`,
`RawGnssEpochSample`, and `FusedLocationSample` to
`LiveEstimatorCoordinator` through `NavigationEventSink`. Its 4,096-event queue is
nonblocking. Queue overflow increments a diagnostic counter instead of blocking an
Android sensor callback indefinitely.

## Core input models

### GNSS observation

`GnssObservation` is a corrected, solver-ready observation:

```text
timestampMillis                  epoch key used by current replay
constellation, svid, signalType  satellite/signal identity
satellitePositionMeters[3]       transmit-time ECEF position
satelliteVelocityMetersPerSecond[3]
correctedPseudorangeMeters       satellite/correction terms already applied
pseudorangeSigmaMeters           positive 1-sigma weight
correctedRangeRateMetersPerSecond
rangeRateSigmaMetersPerSecond    NaN pair means Doppler unavailable
cn0DbHz, elevationDegrees        retained quality metadata
```

`GnssEpoch` groups at least four accepted observations at one epoch. The current
replay uses milliseconds because GSDC is keyed that way. A live adapter must retain
monotonic nanoseconds alongside GNSS time and should not reduce IMU alignment to
millisecond precision internally.

`GnssEpoch.integrationTimestampNanos` returns `elapsedRealtimeNanos` for live
epochs and falls back to checked replay milliseconds only when no Android source
time exists. EKF and inertial-FGO alignment use this property; `timestampMillis`
continues to identify absolute GNSS/dataset time in outputs.

Correction signs follow the current derived-data convention:

```text
corrected pseudorange = raw pseudorange
                      + satellite clock bias
                      - inter-signal bias
                      - ionosphere delay
                      - troposphere delay

corrected range rate = Android pseudorange rate + satellite clock drift
```

Providers must document any different upstream sign convention before mapping into
this type.

### IMU and INS state

`ImuSample` contains monotonic nanoseconds, body-frame specific force in m/s2, and
body-frame angular rate in rad/s.

`InsState` contains:

- monotonic timestamp;
- ENU position and velocity;
- unit quaternion rotating body into ENU;
- body-frame accelerometer and gyroscope biases;
- receiver clock bias in meters and drift in m/s.

`ImuMechanization.propagate(state, previous, current)` requires strictly increasing
sample times and exact equality between the state timestamp and `previous` time.
`preintegrate` requires at least two samples and returns delta position, velocity,
attitude, duration, and sample count.

## Coordinate interface

Implemented functions:

```kotlin
Coordinates.llaToEcef(latitudeDegrees, longitudeDegrees, altitudeMeters)
Coordinates.ecefToLla(positionMeters)
Coordinates.ecefDeltaToEnu(deltaMeters, referenceEcefMeters)

EnuFrame.ecefPositionToEnu(positionEcefMeters)
EnuFrame.enuPositionToEcef(positionEnuMeters)
EnuFrame.ecefVectorToEnu(vectorEcef)
EnuFrame.enuVectorToEcef(vectorEnu)
EnuFrame.rangeDerivativeEcefToEnu(derivativeEcef)
```

Position transforms include the ENU origin translation. Vector and derivative
transforms rotate only. Callers must not use a vector transform for a position.

## Preprocessing interfaces

The current `GsdcReplayEstimator` owns CSV preprocessing internally. It accepts
GPS, GLONASS, QZSS, BeiDou, and Galileo; carrier frequencies from 1.1 to 1.7 GHz;
C/N0 at least 18 dB-Hz; elevation at least 5 degrees; nonzero tracking state;
pseudorange from 1e6 to 1e8 m; and uncertainty no greater than 100 m. Doppler can be
absent while pseudorange remains usable.

Stable rejection reasons are:

```text
not_raw
unsupported_constellation_or_time
invalid_tracking_state
unsupported_frequency
low_signal_or_elevation
missing_pseudorange_input
invalid_pseudorange
```

### Satellite state provider

```kotlin
data class SatelliteSignalKey(
    val constellation: Int,
    val svid: Int,
    val carrierFrequencyHz: Double?,
    val codeType: String,
)

data class SatelliteState(
    val transmitTimeNanos: Long,
    val positionEcefMeters: DoubleArray,
    val velocityEcefMetersPerSecond: DoubleArray,
    val clockBiasMeters: Double,
    val clockDriftMetersPerSecond: Double,
    val health: SatelliteHealth,
    val sourceAgeSeconds: Double,
    val accuracyMeters: Double,
)

interface SatelliteStateProvider {
    fun stateAt(signal: SatelliteSignalKey, transmitTimeNanos: Long): SatelliteStateResult
}
```

`SatelliteStateResult` distinguishes unavailable ephemeris, unhealthy satellite,
stale data, unsupported constellation/signal, invalid time, and successful evaluation.
`SnapshotSatelliteStateProvider` linearly propagates a frozen state for bounded,
deterministic tests. It is not an orbital propagator.

`GpsL1CaEphemerisProvider` is the first production broadcast implementation. It
also implements `NavigationMessageIngestor` and `EphemerisHealthSource`. It accepts
Android type 257, 40-byte, parity-passed/rebuilt subframes 1-3; assembles only
matching IODC/IODE; resolves the modulo-1024 week at query time; and outputs ECEF
position/velocity, satellite clock bias/drift, age, health, and URA accuracy. It
serves GPS L1 C/A only. Other constellations and GPS signals remain **target**.

### Correction provider

```kotlin
data class ObservationCorrections(
    val earthRotationMeters: Double,
    val ionosphereMeters: Double,
    val troposphereMeters: Double,
    val interSignalBiasMeters: Double,
    val uncertaintyMeters: Double,
)

interface CorrectionProvider {
    fun corrections(
        observation: RawGnssObservation,
        satellite: SatelliteState,
        approximateReceiverEcefMeters: DoubleArray,
    ): CorrectionResult
}
```

`ConservativeCorrectionProvider` applies inter-signal bias and a standard-
atmosphere troposphere model when an approximate receiver fix exists. Missing
ionosphere information is zero-mean with a conservative uncertainty budget and
`atmosphericModelComplete=false`; it is not treated as an exact zero correction.
Earth rotation remains in `GnssGeometry` to prevent double application.

### Epoch preprocessor

```kotlin
interface GnssPreprocessor {
    fun consume(clock: RawGnssClock, measurements: List<RawGnssObservation>): PreprocessResult
}
```

`RawGnssPreprocessor.consume(RawGnssEpochSample)` returns zero or one clean
`GnssEpoch`, rejection counts by stable reason, clock-continuity ID, and GNSS
receive time. It never emits a partial epoch with fewer than four pseudorange
observations. Live time construction currently supports GPS, QZSS, Galileo, and
BeiDou, and includes each measurement's `timeOffsetNanos`; GLONASS TOD conversion
remains unsupported and is rejected explicitly. Once a fused reference exists,
satellites below five degrees elevation are rejected before correction/solver use.

`LiveEstimatorCoordinator` resets its backend when the hardware clock
discontinuity counter changes. `KotlinLiveEstimatorBackend` is the shipping
reference implementation: it warm-starts WLS, aligns GNSS updates on monotonic
time, propagates a gravity-aligned EKF at paired IMU rate, and dispatches fixed-lag
FGO to a separate worker. `LiveBackendHealth` exposes pairing skew/drops, WLS/EKF/
FGO updates and failures, stale epochs, resets, the authoritative solution, and
the delayed FGO solution.

The desktop `validation_tools/navgraph_replay.py` reader implements the same
GPS L1 C/A preprocessing contract for schema-v1 NDJSON. It groups each clock with
raw measurements by exact `elapsed_realtime_nanos`, partitions discontinuity arcs,
and processes navigation/fused events causally. Its `clean_observations.csv` is a
portable inspection boundary; `solutions.csv` records the corresponding monotonic
time and clock arc so GNSS-time collisions cannot silently join reset segments.

## Estimator interfaces

### WLS

```kotlin
GnssWlsSolver(applyEarthRotation: Boolean = true)
    .solve(epochs: List<GnssEpoch>): List<PositionEstimate>
```

The solver carries the previous epoch as initialization, uses robust pseudorange
position/clock iterations, and then solves velocity/clock drift from valid Doppler.
`PositionEstimate` exposes convergence, iteration count, observations, residual
rejection count, and measured latency.

### GNSS/INS EKF

```kotlin
val ekf = GnssInsEkf(frame, initialState, initialCovariance)
ekf.propagate(previousImu, currentImu)
ekf.update(epoch)
```

The nominal state is `InsState`; covariance is a 17 by 17 local error covariance in
the order position, velocity, attitude error, accelerometer bias, gyroscope bias,
clock bias, and clock drift. `EkfHealth` exposes accepted/rejected pseudorange and
Doppler counts, propagation count, non-monotonic IMU count, native propagation and
GNSS update counts, and native failure count. `GnssInsEkf.engine` reports the engine
used by the latest successful step or `KOTLIN FALLBACK` after a failed native step.

The caller owns initialization and chronological dispatch. A GNSS update whose
timestamp differs from the propagated state by more than 1 ms is rejected by
contract.

### Fixed-lag smoother

```kotlin
InertialFixedLagSmoother(
    frame,
    lagSeconds = FgoConfig.DEFAULT_LAG_SECONDS,
    maxKeyframes = FgoConfig.MAX_KEYFRAMES,
).run(epochs, imuSamples, wls, latestOnly = false): InertialSmootherResult
```

Inputs must be chronologically ordered, `epochs.size == wls.size`, and each adjacent
GNSS interval must contain at least two IMU samples. Current graph state is ENU
position/velocity, accelerometer bias, clock bias, and clock drift. Output includes
one estimate per solved window and the number of inertial factors; `latestOnly = true`
solves only the newest window and is the live path. The default horizon is five
seconds with a twelve-keyframe cap, so the dense matrix is bounded at the expected
1 Hz GNSS rate.

`InertialSmootherResult.stateUncertainties` exposes final-state ENU position and
velocity plus clock marginal sigmas from selected diagonal entries of the robust
normal-equation inverse. The live adapter uses these values with the captured EKF
same-keyframe covariance, a four-times correlation inflation, a 99.9% six-state
normalized innovation gate, absolute correction bounds, and a two-second age gate.
Accepted correlated corrections use gain 0.25 and leave EKF covariance unchanged.
The result also reports the latest engine, number of native windows, native
failures, window solves, converged windows, and the maximum keyframes in any window.
A native failure is followed by the Kotlin reference solve for that window.

A future incremental smoother must additionally expose keyframe insertion,
marginalization status, and a reset operation at clock/frame discontinuities.

## Geometry and residual interface

`GnssGeometry.predict` consumes receiver ECEF state, clock state, and a clean
observation. It returns predicted pseudorange/range rate plus ECEF derivatives.
Earth rotation is enabled by default. Residual sign conventions are:

```text
least-squares/FGO residual = predicted - observed
EKF innovation            = observed - predicted
```

Any new factor must include a central-difference Jacobian test at representative
ECEF magnitudes and must state its perturbation frame.

## Replay interface

```kotlin
interface ReplayEstimator {
    fun run(): ReplayResult
}
```

`AssetReplayEstimator` is the Android implementation. A result includes dataset
identity, observation/rejection counts, WLS/FGO metrics and trajectories, numerical
Jacobian errors, optional synthetic GNSS/INS validation, and total compute latency.
Replay must be deterministic except for timing fields.

## JNI boundary

The stateless WLS JNI call flattens
each observation into ten doubles in this fixed order:

```text
satellite ECEF position x/y/z
satellite ECEF velocity x/y/z
corrected pseudorange, pseudorange sigma
corrected range rate, range-rate sigma
```

The initial receiver state is eight doubles: ECEF position[3], ECEF velocity[3],
clock bias, and clock drift. Native output is thirteen doubles: ECEF position[3],
ECEF velocity[3], clock bias, clock drift, iteration count, observation count,
rejected residual count, convergence flag, and numerical-validity flag. Invalid
dimensions throw a Java argument error; non-finite solutions return a false validity
flag. No C++ exception crosses JNI. The C++ API remains free of Android types and
uses the same predicted-minus-observed residual convention as Kotlin.

`NativeGnssWls` loads `navgraph_estimator` optionally. The live backend accepts a
finite successful native result, otherwise falls back to Kotlin and records the
failure. This preserves a working logger/estimator APK when no native library is
packaged.

The implemented nominal INS JNI call carries timestamps as `Long`, never doubles.
Its state payload has 18 doubles in this order:

```text
ENU position[3], ENU velocity[3]
body-to-ENU quaternion w/x/y/z
body accelerometer bias[3], body gyro bias[3]
receiver clock bias, receiver clock drift
```

Each endpoint IMU payload has body specific force[3] followed by body angular
rate[3]. The 19-double result returns the same state layout plus a typed status:
success, non-increasing timestamp, state/sample timestamp mismatch, non-finite
input, or invalid quaternion. Live integration accepts only status success, exact
current timestamp, and a finite full state. Otherwise it increments native INS
failure telemetry and runs the Kotlin nominal propagator.

The complete stateless EKF JNI calls retain timestamps as `Long` and reuse the
18-double nominal-state and six-double endpoint-IMU layouts above. Covariance is a
row-major 289-double array in the documented 17-state order. GNSS update additionally
receives a 12-double frame payload:

```text
ENU origin ECEF position[3]
row-major ECEF-to-ENU rotation[9]
```

GNSS observations reuse the WLS stride-10 layout. Both propagation and update return
313 doubles: nominal state[18], covariance[289], typed status, propagation count,
accepted/rejected pseudorange counts, and accepted/rejected Doppler counts. Statuses
distinguish success, non-increasing IMU time, state/sample mismatch, non-finite input,
invalid quaternion, GNSS/state time mismatch, and invalid local frame. Live
integration accepts only success with exact propagation time, finite state, symmetric
finite covariance, and nonnegative covariance diagonal. Otherwise it records an EKF
failure and runs the Kotlin reference step.

The fixed-lag JNI call sends one complete window. Inputs are the same 12-double local
frame, epoch timestamps as `Long`, cumulative observation offsets, stride-10 GNSS
observations, eight WLS seed values per keyframe, and seven preintegration values per
interval:

```text
delta time
delta ENU position[3]
delta ENU velocity[3]
```

Each graph keyframe has 11 doubles: ENU position[3], ENU velocity[3], accelerometer
bias[3], clock bias, and clock drift. The variable-length result returns every
keyframe followed by 13 values: iterations, convergence, typed status, skipped-factor
count, marginal-valid flag, ENU position sigma[3], ENU velocity sigma[3], clock-bias
sigma, and clock-drift sigma. Host parity covers all keyframe components and all
eight marginals. Live integration accepts a successful, dimensionally valid finite
result; otherwise it records a native window failure and solves the same graph in
Kotlin.

The future streaming NDK boundary remains **target**. It should use versioned,
length-delimited messages or flat plain buffers and provide create, reset,
push-IMU, push-GNSS-epoch, poll-state, and destroy operations. Kotlin remains
responsible for permissions, Android callbacks, service lifecycle, files, and UI.
