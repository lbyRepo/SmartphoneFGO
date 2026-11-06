package com.example.navgraph.estimator

import java.io.File
import org.junit.Assume.assumeTrue
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class NativeGnssInsEkfParityTest {
    @Test
    fun jniMatchesKotlinPropagationAndSequentialGnssUpdate() {
        val library = File(
            System.getProperty("java.io.tmpdir"),
            "navgraph_native_parity/libnavgraph_estimator.dylib",
        )
        assumeTrue("Native parity library is not built: ${library.absolutePath}", library.isFile)
        NativeGnssInsEkf.loadAbsoluteForTests(library.absolutePath)
        val dataset = SyntheticGnssInsDatasetFactory.create(seed = 91L)
        val initialCovariance = GnssInsEkf.defaultInitialCovariance().also { covariance ->
            covariance[0][3] = 0.7
            covariance[3][0] = 0.7
            covariance[6][12] = -0.001
            covariance[12][6] = -0.001
        }
        val state = InsState(
            timestampNanos = dataset.imuSamples[3].timestampNanos,
            positionEnuMeters = doubleArrayOf(4.0, -3.0, 2.0),
            velocityEnuMetersPerSecond = doubleArrayOf(0.5, -0.2, 0.1),
            attitudeBodyToEnu = UnitQuaternion.fromRotationVector(
                doubleArrayOf(0.02, -0.04, 0.03),
            ),
            accelerometerBiasBodyMetersPerSecondSquared = doubleArrayOf(0.02, -0.01, 0.03),
            gyroBiasBodyRadiansPerSecond = doubleArrayOf(0.001, -0.002, 0.003),
            clockBiasMeters = 80.0,
            clockDriftMetersPerSecond = 0.2,
        )
        val previous = dataset.imuSamples[3]
        val current = dataset.imuSamples[4]
        val kotlinPropagation = GnssInsEkf(
            dataset.frame,
            state,
            initialCovariance,
            applyEarthRotation = false,
        ).also { it.propagate(previous, current) }
        val nativePropagation = requireNotNull(
            NativeGnssInsEkf.propagate(state, initialCovariance, previous, current),
        )
        assertEquals(NativeEkfStatus.OK, nativePropagation.status)
        assertEquals(1, nativePropagation.health.propagationSteps)
        assertStateClose(kotlinPropagation.state, nativePropagation.state, 2.0e-12)
        assertCovarianceClose(
            kotlinPropagation.covariance,
            nativePropagation.covariance,
            3.0e-13,
        )

        val epoch = dataset.gnssEpochs.first().copy(
            elapsedRealtimeNanos = dataset.truthStates.first().timestampNanos,
        )
        val updateState = state.copy(
            timestampNanos = epoch.integrationTimestampNanos,
            positionEnuMeters = doubleArrayOf(7.0, -5.0, 3.0),
            velocityEnuMetersPerSecond = doubleArrayOf(1.3, -0.4, 0.2),
            clockBiasMeters = 70.0,
            clockDriftMetersPerSecond = 0.5,
        )
        val kotlinUpdate = GnssInsEkf(
            dataset.frame,
            updateState,
            initialCovariance,
            applyEarthRotation = false,
        ).also { it.update(epoch) }
        val nativeUpdate = requireNotNull(
            NativeGnssInsEkf.update(
                dataset.frame,
                updateState,
                initialCovariance,
                epoch,
                applyEarthRotation = false,
            ),
        )
        assertEquals(NativeEkfStatus.OK, nativeUpdate.status)
        assertEquals(kotlinUpdate.health.acceptedPseudorange,
            nativeUpdate.health.acceptedPseudorange)
        assertEquals(kotlinUpdate.health.rejectedPseudorange,
            nativeUpdate.health.rejectedPseudorange)
        assertEquals(kotlinUpdate.health.acceptedDoppler, nativeUpdate.health.acceptedDoppler)
        assertEquals(kotlinUpdate.health.rejectedDoppler, nativeUpdate.health.rejectedDoppler)
        assertTrue(nativeUpdate.health.acceptedPseudorange > 0)
        assertTrue(nativeUpdate.health.acceptedDoppler > 0)
        assertStateClose(kotlinUpdate.state, nativeUpdate.state, 2.0e-8)
        assertCovarianceClose(kotlinUpdate.covariance, nativeUpdate.covariance, 2.0e-8)

        val mistimed = requireNotNull(
            NativeGnssInsEkf.update(
                dataset.frame,
                updateState,
                initialCovariance,
                epoch.copy(elapsedRealtimeNanos = epoch.integrationTimestampNanos + 1_000_001L),
                applyEarthRotation = false,
            ),
        )
        assertEquals(NativeEkfStatus.GNSS_TIMESTAMP_MISMATCH, mistimed.status)

        val invalidEpoch = epoch.copy(
            observations = listOf(
                epoch.observations.first().copy(
                    satellitePositionMeters = doubleArrayOf(Double.NaN, 0.0, 0.0),
                ),
            ),
        )
        val kotlinInvalid = GnssInsEkf(
            dataset.frame,
            updateState,
            initialCovariance,
            applyEarthRotation = false,
        ).also { it.update(invalidEpoch) }
        val nativeInvalid = requireNotNull(
            NativeGnssInsEkf.update(
                dataset.frame,
                updateState,
                initialCovariance,
                invalidEpoch,
                applyEarthRotation = false,
            ),
        )
        assertEquals(1, kotlinInvalid.health.rejectedPseudorange)
        assertEquals(1, kotlinInvalid.health.rejectedDoppler)
        assertEquals(kotlinInvalid.health.rejectedPseudorange,
            nativeInvalid.health.rejectedPseudorange)
        assertEquals(kotlinInvalid.health.rejectedDoppler, nativeInvalid.health.rejectedDoppler)
        assertStateClose(kotlinInvalid.state, nativeInvalid.state, 0.0)
        assertCovarianceClose(kotlinInvalid.covariance, nativeInvalid.covariance, 0.0)
    }

    private fun assertStateClose(expected: InsState, actual: InsState, tolerance: Double) {
        assertEquals(expected.timestampNanos, actual.timestampNanos)
        expected.positionEnuMeters.indices.forEach { axis ->
            assertEquals(expected.positionEnuMeters[axis], actual.positionEnuMeters[axis], tolerance)
            assertEquals(expected.velocityEnuMetersPerSecond[axis],
                actual.velocityEnuMetersPerSecond[axis], tolerance)
            assertEquals(expected.accelerometerBiasBodyMetersPerSecondSquared[axis],
                actual.accelerometerBiasBodyMetersPerSecondSquared[axis], tolerance)
            assertEquals(expected.gyroBiasBodyRadiansPerSecond[axis],
                actual.gyroBiasBodyRadiansPerSecond[axis], tolerance)
        }
        assertEquals(expected.attitudeBodyToEnu.w, actual.attitudeBodyToEnu.w, tolerance)
        assertEquals(expected.attitudeBodyToEnu.x, actual.attitudeBodyToEnu.x, tolerance)
        assertEquals(expected.attitudeBodyToEnu.y, actual.attitudeBodyToEnu.y, tolerance)
        assertEquals(expected.attitudeBodyToEnu.z, actual.attitudeBodyToEnu.z, tolerance)
        assertEquals(expected.clockBiasMeters, actual.clockBiasMeters, tolerance)
        assertEquals(expected.clockDriftMetersPerSecond,
            actual.clockDriftMetersPerSecond, tolerance)
    }

    private fun assertCovarianceClose(
        expected: Array<DoubleArray>,
        actual: Array<DoubleArray>,
        tolerance: Double,
    ) {
        expected.indices.forEach { row ->
            expected[row].indices.forEach { column ->
                assertEquals(expected[row][column], actual[row][column], tolerance,
                    "covariance[$row][$column]")
            }
        }
    }
}
