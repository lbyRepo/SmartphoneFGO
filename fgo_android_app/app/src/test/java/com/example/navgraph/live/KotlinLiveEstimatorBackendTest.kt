package com.example.navgraph.live

import com.example.navgraph.estimator.SyntheticGnssInsDatasetFactory
import java.util.concurrent.Executors
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class KotlinLiveEstimatorBackendTest {
    @Test
    fun syntheticStreamUsesElapsedTimeForLiveEkfAndRunsFgo() {
        val dataset = SyntheticGnssInsDatasetFactory.create()
        val executor = Executors.newSingleThreadExecutor()
        val backend = KotlinLiveEstimatorBackend(
            reorderDelayNanos = 0L,
            smootherExecutor = executor,
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
                    val feedbackDeadline = System.nanoTime() + 5_000_000_000L
                    while (backend.backendHealth().fgoUpdates == 0L &&
                        System.nanoTime() < feedbackDeadline
                    ) {
                        Thread.sleep(5L)
                    }
                }
            }

            val deadline = System.nanoTime() + 5_000_000_000L
            while (backend.backendHealth().fgoUpdates == 0L && System.nanoTime() < deadline) {
                Thread.sleep(10L)
            }
            val health = backend.backendHealth()
            assertEquals(2_001L, health.pairedImuSamples)
            assertEquals(21L, health.wlsUpdates)
            assertEquals(21L, health.ekfGnssUpdates)
            assertTrue(health.ekfPropagationUpdates >= 2_000L)
            assertEquals(0L, health.staleGnssUpdates)
            assertTrue(health.fgoUpdates > 0L, "FGO did not publish before the deadline: $health")
            assertTrue(
                health.fgoFeedbackAccepted > 0L,
                "No covariance-gated FGO correction was accepted: $health",
            )
            assertEquals(LiveSolutionSource.EKF, assertNotNull(health.latestSolution).source)
            assertTrue(health.latestSolution.latitudeDegrees.isFinite())
            assertEquals(LiveSolutionSource.FGO, assertNotNull(health.latestFgoSolution).source)
        } finally {
            backend.close()
        }
    }

    @Test
    fun imuPairingAccountsForSkewAndDiscardedSamples() {
        val assembler = ImuSampleAssembler(maximumSkewNanos = 20_000_000L)
        assertTrue(assembler.push(rawImu(11_000_000L, ImuSensorType.GYROSCOPE, doubleArrayOf(1.0, 2.0, 3.0))).isEmpty())
        val paired = assembler.push(rawImu(
            10_000_000L,
            ImuSensorType.ACCELEROMETER,
            doubleArrayOf(4.0, 5.0, 6.0),
        ))
        assertEquals(1, paired.size)
        assertEquals(10_000_000L, paired.single().timestampNanos)
        assertEquals(1_000_000L, assembler.maximumPairSkewNanos)

        assembler.push(rawImu(100_000_000L, ImuSensorType.GYROSCOPE, DoubleArray(3)))
        assembler.push(rawImu(200_000_000L, ImuSensorType.ACCELEROMETER, DoubleArray(3)))
        assertTrue(assembler.discardedSamples > 0L)
    }

    @Test
    fun resetClearsPublishedStateAndStartsANewGeneration() {
        val backend = KotlinLiveEstimatorBackend(smootherExecutor = Executors.newSingleThreadExecutor())
        try {
            backend.reset(7)
            val health = backend.backendHealth()
            assertEquals(1L, health.resetCount)
            assertEquals("reset_clock_arc_7", health.state)
            assertEquals(null, health.latestSolution)
            assertEquals(null, health.latestFgoSolution)
        } finally {
            backend.close()
        }
    }

    private fun rawImu(timestampNanos: Long, type: ImuSensorType, values: DoubleArray) =
        RawImuSample(
            elapsedRealtimeNanos = timestampNanos,
            callbackElapsedRealtimeNanos = timestampNanos,
            sensorType = type,
            values = values.copyOf(),
            accuracy = 3,
        )
}
