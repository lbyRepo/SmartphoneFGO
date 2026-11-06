package com.example.navgraph.estimator

import java.io.File
import org.junit.Assume.assumeTrue
import kotlin.test.Test
import kotlin.test.assertEquals

class NativeInsMechanizationParityTest {
    @Test
    fun jniAndKotlinMechanizationMatchForBiasedRotatingState() {
        val library = File(
            System.getProperty("java.io.tmpdir"),
            "navgraph_native_parity/libnavgraph_estimator.dylib",
        )
        assumeTrue("Native parity library is not built: ${library.absolutePath}", library.isFile)
        NativeInsMechanization.loadAbsoluteForTests(library.absolutePath)
        val state = InsState(
            timestampNanos = 5_000_000_000L,
            positionEnuMeters = doubleArrayOf(12.0, -4.0, 8.0),
            velocityEnuMetersPerSecond = doubleArrayOf(0.7, -1.2, 0.3),
            attitudeBodyToEnu = UnitQuaternion.fromRotationVector(doubleArrayOf(0.1, -0.2, 0.3)),
            accelerometerBiasBodyMetersPerSecondSquared = doubleArrayOf(0.02, -0.01, 0.03),
            gyroBiasBodyRadiansPerSecond = doubleArrayOf(0.001, -0.002, 0.003),
            clockBiasMeters = 45.0,
            clockDriftMetersPerSecond = 0.4,
        )
        val previous = ImuSample(
            state.timestampNanos,
            doubleArrayOf(0.3, -0.4, 9.9),
            doubleArrayOf(0.02, -0.03, 0.04),
        )
        val current = ImuSample(
            state.timestampNanos + 10_000_000L,
            doubleArrayOf(0.32, -0.38, 9.88),
            doubleArrayOf(0.021, -0.029, 0.041),
        )
        val kotlin = ImuMechanization.propagate(state, previous, current)
        val native = requireNotNull(NativeInsMechanization.propagate(state, previous, current))
        assertEquals(NativeInsStatus.OK, native.status)
        assertStateClose(kotlin, native.state, 2.0e-12)

        val duplicate = requireNotNull(
            NativeInsMechanization.propagate(state, previous, previous),
        )
        assertEquals(NativeInsStatus.NON_INCREASING_TIMESTAMP, duplicate.status)
        assertStateClose(state, duplicate.state, 0.0)
    }

    private fun assertStateClose(expected: InsState, actual: InsState, tolerance: Double) {
        assertEquals(expected.timestampNanos, actual.timestampNanos)
        expected.positionEnuMeters.indices.forEach { axis ->
            assertEquals(expected.positionEnuMeters[axis], actual.positionEnuMeters[axis], tolerance)
            assertEquals(
                expected.velocityEnuMetersPerSecond[axis],
                actual.velocityEnuMetersPerSecond[axis],
                tolerance,
            )
            assertEquals(
                expected.accelerometerBiasBodyMetersPerSecondSquared[axis],
                actual.accelerometerBiasBodyMetersPerSecondSquared[axis],
                tolerance,
            )
            assertEquals(
                expected.gyroBiasBodyRadiansPerSecond[axis],
                actual.gyroBiasBodyRadiansPerSecond[axis],
                tolerance,
            )
        }
        assertEquals(expected.attitudeBodyToEnu.w, actual.attitudeBodyToEnu.w, tolerance)
        assertEquals(expected.attitudeBodyToEnu.x, actual.attitudeBodyToEnu.x, tolerance)
        assertEquals(expected.attitudeBodyToEnu.y, actual.attitudeBodyToEnu.y, tolerance)
        assertEquals(expected.attitudeBodyToEnu.z, actual.attitudeBodyToEnu.z, tolerance)
        assertEquals(expected.clockBiasMeters, actual.clockBiasMeters, tolerance)
        assertEquals(
            expected.clockDriftMetersPerSecond,
            actual.clockDriftMetersPerSecond,
            tolerance,
        )
    }
}
