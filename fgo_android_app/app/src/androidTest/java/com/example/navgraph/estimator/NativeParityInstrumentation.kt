package com.example.navgraph.estimator

import android.app.Activity
import android.app.Instrumentation
import android.os.Bundle
import com.example.navgraph.live.ImuSensorType
import com.example.navgraph.live.KotlinLiveEstimatorBackend
import com.example.navgraph.live.RawImuSample
import java.util.concurrent.Executors
import kotlin.math.sqrt

class NativeParityInstrumentation : Instrumentation() {
    override fun onCreate(arguments: Bundle?) {
        super.onCreate(arguments)
        start()
    }

    override fun onStart() {
        check(NativeGnssWls.isAvailable) { "Packaged navgraph_estimator library did not load" }
        check(NativeGnssInsEkf.isAvailable) { "Packaged native EKF did not load" }
        check(NativeFixedLagSmoother.isAvailable) { "Packaged native FGO did not load" }
        val (epoch, truth) = fixture()
        val native = checkNotNull(NativeGnssWls.solveEpoch(
            epoch,
            applyEarthRotation = false,
        ))
        check(native.numericallyValid && native.estimate.converged) {
            "Native WLS failed: $native"
        }
        val positionError = distance(
            native.estimate.positionEcefMeters,
            truth.positionEcefMeters,
        )
        val velocityError = distance(
            native.estimate.velocityEcefMetersPerSecond,
            truth.velocityEcefMetersPerSecond,
        )
        check(positionError < 1.0e-3) { "Native position error $positionError m" }
        check(velocityError < 1.0e-4) { "Native velocity error $velocityError m/s" }
        val liveHealth = runLiveBackend()
        finish(Activity.RESULT_OK, Bundle().apply {
            putString("native_library", "loaded")
            putString("abi", android.os.Build.SUPPORTED_ABIS.firstOrNull().orEmpty())
            putDouble("position_error_m", positionError)
            putDouble("velocity_error_mps", velocityError)
            putInt("iterations", native.estimate.iterations)
            putInt("observations", native.estimate.observationCount)
            putString("live_wls_engine", liveHealth.wlsEngine)
            putLong("live_wls_updates", liveHealth.wlsUpdates)
            putLong("live_native_failures", liveHealth.nativeWlsFailures)
            putString("live_ins_engine", liveHealth.insEngine)
            putLong("live_native_ins_propagations", liveHealth.nativeInsPropagationUpdates)
            putLong("live_native_ins_failures", liveHealth.nativeInsFailures)
            putString("live_ekf_engine", liveHealth.ekfEngine)
            putLong("live_native_ekf_propagations", liveHealth.nativeEkfPropagationUpdates)
            putLong("live_native_ekf_gnss", liveHealth.nativeEkfGnssUpdates)
            putLong("live_native_ekf_failures", liveHealth.nativeEkfFailures)
            putLong("live_ekf_propagations", liveHealth.ekfPropagationUpdates)
            putLong("live_fgo_updates", liveHealth.fgoUpdates)
            putString("live_fgo_engine", liveHealth.fgoEngine)
            putLong("live_native_fgo_windows", liveHealth.nativeFgoWindowUpdates)
            putLong("live_native_fgo_failures", liveHealth.nativeFgoFailures)
            putLong("live_fgo_feedback_accepted", liveHealth.fgoFeedbackAccepted)
        })
    }

    private fun runLiveBackend(): com.example.navgraph.live.LiveBackendHealth {
        val dataset = SyntheticGnssInsDatasetFactory.create()
        val backend = KotlinLiveEstimatorBackend(
            reorderDelayNanos = 0L,
            smootherExecutor = Executors.newSingleThreadExecutor(),
            applyEarthRotation = false,
        )
        try {
            val epochs = dataset.gnssEpochs.mapIndexed { index, epoch ->
                epoch.copy(
                    timestampMillis = 1_300_000_000_000L + index * 1_000L,
                    elapsedRealtimeNanos = epoch.integrationTimestampNanos,
                )
            }
            val epochsByElapsed = epochs.associateBy { it.integrationTimestampNanos }
            dataset.imuSamples.forEach { imu ->
                epochsByElapsed[imu.timestampNanos]?.let(backend::pushGnssEpoch)
                backend.pushImu(rawImu(
                    imu.timestampNanos,
                    ImuSensorType.ACCELEROMETER,
                    imu.specificForceBodyMetersPerSecondSquared,
                ))
                backend.pushImu(rawImu(
                    imu.timestampNanos,
                    ImuSensorType.GYROSCOPE,
                    imu.angularRateBodyRadiansPerSecond,
                ))
                if (imu.timestampNanos == epochs[1].integrationTimestampNanos) {
                    waitUntil { backend.backendHealth().fgoUpdates > 0L }
                }
            }
            waitUntil { backend.backendHealth().fgoUpdates > 0L }
            return backend.backendHealth().also { health ->
                check(health.wlsEngine == "C++ JNI") { "Unexpected WLS engine: $health" }
                check(health.nativeWlsFailures == 0L) { "Native fallback occurred: $health" }
                check(health.wlsUpdates == 21L && health.ekfGnssUpdates == 21L) {
                    "Incomplete live GNSS processing: $health"
                }
                check(health.ekfPropagationUpdates >= 2_000L) {
                    "Incomplete live IMU propagation: $health"
                }
                check(health.insEngine == "C++ JNI") { "Unexpected INS engine: $health" }
                check(health.nativeInsPropagationUpdates >= 2_000L &&
                    health.nativeInsFailures == 0L
                ) {
                    "Native INS propagation fallback occurred: $health"
                }
                check(health.ekfEngine == "C++ JNI") { "Unexpected EKF engine: $health" }
                check(health.nativeEkfPropagationUpdates >= 2_000L &&
                    health.nativeEkfGnssUpdates == 21L && health.nativeEkfFailures == 0L
                ) {
                    "Native EKF fallback occurred: $health"
                }
                check(health.fgoFeedbackAccepted > 0L) {
                    "No live FGO correction accepted: $health"
                }
                check(health.fgoEngine == "C++ JNI" &&
                    health.nativeFgoWindowUpdates > 0L && health.nativeFgoFailures == 0L
                ) {
                    "Native FGO fallback occurred: $health"
                }
            }
        } finally {
            backend.close()
        }
    }

    private fun waitUntil(condition: () -> Boolean) {
        val deadline = System.nanoTime() + 5_000_000_000L
        while (!condition() && System.nanoTime() < deadline) Thread.sleep(5L)
        check(condition()) { "Timed out waiting for asynchronous live estimator" }
    }

    private fun rawImu(timestampNanos: Long, type: ImuSensorType, values: DoubleArray) =
        RawImuSample(
            elapsedRealtimeNanos = timestampNanos,
            callbackElapsedRealtimeNanos = timestampNanos,
            sensorType = type,
            values = values.copyOf(),
            accuracy = 3,
        )

    private fun fixture(): Pair<GnssEpoch, ReceiverStateFixture> {
        val truth = ReceiverStateFixture(
            positionEcefMeters = doubleArrayOf(-1_500_000.0, 6_190_000.0, 150_000.0),
            velocityEcefMetersPerSecond = doubleArrayOf(12.0, -3.0, 0.8),
            clockBiasMeters = 87.5,
            clockDriftMetersPerSecond = 0.22,
        )
        val positions = listOf(
            doubleArrayOf(15_600_000.0, 7_540_000.0, 20_140_000.0),
            doubleArrayOf(18_760_000.0, 2_750_000.0, 18_610_000.0),
            doubleArrayOf(17_610_000.0, 14_630_000.0, 13_480_000.0),
            doubleArrayOf(19_170_000.0, 610_000.0, 18_390_000.0),
            doubleArrayOf(17_800_000.0, -13_200_000.0, 14_800_000.0),
            doubleArrayOf(21_100_000.0, 9_000_000.0, -12_000_000.0),
            doubleArrayOf(-15_000_000.0, 18_000_000.0, 14_000_000.0),
            doubleArrayOf(-17_000_000.0, -13_000_000.0, 16_000_000.0),
        )
        val velocities = listOf(
            doubleArrayOf(-900.0, 2_100.0, -400.0),
            doubleArrayOf(-1_200.0, 1_700.0, 600.0),
            doubleArrayOf(-1_500.0, -900.0, 1_800.0),
            doubleArrayOf(-700.0, 2_300.0, -300.0),
            doubleArrayOf(1_400.0, 1_200.0, -1_000.0),
            doubleArrayOf(-1_800.0, 500.0, 1_100.0),
            doubleArrayOf(900.0, -1_500.0, 1_300.0),
            doubleArrayOf(1_200.0, 800.0, -1_700.0),
        )
        val observations = positions.indices.map { index ->
            val delta = subtract(truth.positionEcefMeters, positions[index])
            val range = norm(delta)
            val lineOfSight = DoubleArray(3) { delta[it] / range }
            val relativeVelocity = subtract(
                truth.velocityEcefMetersPerSecond,
                velocities[index],
            )
            GnssObservation(
                timestampMillis = 12_345L,
                constellation = 1,
                svid = index + 1,
                signalType = "SYNTHETIC_L1",
                satellitePositionMeters = positions[index],
                satelliteVelocityMetersPerSecond = velocities[index],
                correctedPseudorangeMeters = range + truth.clockBiasMeters,
                pseudorangeSigmaMeters = 2.5,
                correctedRangeRateMetersPerSecond = dot(lineOfSight, relativeVelocity) +
                    truth.clockDriftMetersPerSecond,
                rangeRateSigmaMetersPerSecond = 0.15,
                cn0DbHz = 40.0,
                elevationDegrees = 35.0,
            )
        }
        return GnssEpoch(12_345L, observations, 12_345_000_000L) to truth
    }

    private fun distance(first: DoubleArray, second: DoubleArray): Double =
        norm(subtract(first, second))

    private fun subtract(first: DoubleArray, second: DoubleArray): DoubleArray =
        DoubleArray(first.size) { first[it] - second[it] }

    private fun dot(first: DoubleArray, second: DoubleArray): Double =
        first.indices.sumOf { first[it] * second[it] }

    private fun norm(values: DoubleArray): Double = sqrt(dot(values, values))

    private data class ReceiverStateFixture(
        val positionEcefMeters: DoubleArray,
        val velocityEcefMetersPerSecond: DoubleArray,
        val clockBiasMeters: Double,
        val clockDriftMetersPerSecond: Double,
    )
}
