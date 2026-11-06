package com.example.navgraph.estimator

import java.io.File
import org.junit.Assume.assumeTrue
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class NativeGnssWlsParityTest {
    @Test
    fun jniAndKotlinWlsMatchOnTheSameEpoch() {
        val library = File(
            System.getProperty("java.io.tmpdir"),
            "navgraph_native_parity/libnavgraph_estimator.dylib",
        )
        assumeTrue("Native parity library is not built: ${library.absolutePath}", library.isFile)
        NativeGnssWls.loadAbsoluteForTests(library.absolutePath)
        val fixture = fixture()
        val kotlin = GnssWlsSolver(applyEarthRotation = false).solveEpoch(fixture.first)
        val native = assertNotNull(NativeGnssWls.solveEpoch(
            fixture.first,
            applyEarthRotation = false,
        ))
        val expected = fixture.second

        assertTrue(kotlin.converged)
        assertTrue(native.estimate.converged)
        assertTrue(native.numericallyValid)
        assertVectorClose(kotlin.positionEcefMeters, native.estimate.positionEcefMeters, 1.0e-6)
        assertVectorClose(kotlin.velocityEcefMetersPerSecond,
            native.estimate.velocityEcefMetersPerSecond, 1.0e-7)
        assertEquals(kotlin.clockBiasMeters, native.estimate.clockBiasMeters, 1.0e-6)
        assertEquals(kotlin.clockDriftMetersPerSecond,
            native.estimate.clockDriftMetersPerSecond, 1.0e-7)
        assertEquals(kotlin.iterations, native.estimate.iterations)
        assertEquals(kotlin.rejectedCount, native.estimate.rejectedCount)
        assertVectorClose(expected.positionEcefMeters, native.estimate.positionEcefMeters, 1.0e-3)
        assertVectorClose(expected.velocityEcefMetersPerSecond,
            native.estimate.velocityEcefMetersPerSecond, 1.0e-4)
        println(
            "native parity: position=${native.estimate.positionEcefMeters.contentToString()}, " +
                "velocity=${native.estimate.velocityEcefMetersPerSecond.contentToString()}, " +
                "iterations=${native.estimate.iterations}",
        )
    }

    private fun fixture(): Pair<GnssEpoch, ReceiverStateFixture> {
        val truth = ReceiverStateFixture(
            doubleArrayOf(-1_500_000.0, 6_190_000.0, 150_000.0),
            doubleArrayOf(12.0, -3.0, 0.8),
            87.5,
            0.22,
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
            val distance = norm(delta)
            val lineOfSight = DoubleArray(3) { delta[it] / distance }
            val relativeVelocity = subtract(truth.velocityEcefMetersPerSecond, velocities[index])
            GnssObservation(
                timestampMillis = 12_345L,
                constellation = 1,
                svid = index + 1,
                signalType = "SYNTHETIC_L1",
                satellitePositionMeters = positions[index],
                satelliteVelocityMetersPerSecond = velocities[index],
                correctedPseudorangeMeters = distance + truth.clockBiasMeters,
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

    private fun dot(left: DoubleArray, right: DoubleArray): Double =
        left.indices.sumOf { left[it] * right[it] }

    private fun norm(values: DoubleArray): Double = sqrt(dot(values, values))

    private fun assertVectorClose(expected: DoubleArray, actual: DoubleArray, tolerance: Double) {
        expected.indices.forEach { assertEquals(expected[it], actual[it], tolerance) }
    }

    private data class ReceiverStateFixture(
        val positionEcefMeters: DoubleArray,
        val velocityEcefMetersPerSecond: DoubleArray,
        val clockBiasMeters: Double,
        val clockDriftMetersPerSecond: Double,
    )
}
