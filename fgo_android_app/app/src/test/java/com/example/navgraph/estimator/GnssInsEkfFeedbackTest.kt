package com.example.navgraph.estimator

import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals

class GnssInsEkfFeedbackTest {
    @Test
    fun correlatedSmootherDeltaMovesTranslationWithoutReducingCovariance() {
        val frame = EnuFrame(Coordinates.llaToEcef(1.3521, 103.8198, 35.0))
        val ekf = GnssInsEkf(
            frame,
            InsState(
                timestampNanos = 10_000_000_000L,
                positionEnuMeters = doubleArrayOf(1.0, 2.0, 3.0),
                velocityEnuMetersPerSecond = doubleArrayOf(0.1, 0.2, 0.3),
                attitudeBodyToEnu = UnitQuaternion.IDENTITY,
                accelerometerBiasBodyMetersPerSecondSquared = DoubleArray(3),
                gyroBiasBodyRadiansPerSecond = DoubleArray(3),
                clockBiasMeters = 20.0,
                clockDriftMetersPerSecond = 0.5,
            ),
        )
        val covarianceBefore = ekf.covariance.map(DoubleArray::copyOf).toTypedArray()

        ekf.applyCorrelatedSmootherDelta(
            positionDeltaEnuMeters = doubleArrayOf(4.0, -2.0, 1.0),
            velocityDeltaEnuMetersPerSecond = doubleArrayOf(0.4, -0.2, 0.1),
            clockBiasDeltaMeters = 8.0,
            clockDriftDeltaMetersPerSecond = 0.4,
            gain = 0.25,
        )

        doubleArrayOf(2.0, 1.5, 3.25).indices.forEach { axis ->
            assertEquals(
                doubleArrayOf(2.0, 1.5, 3.25)[axis],
                ekf.state.positionEnuMeters[axis],
                1.0e-12,
            )
            assertEquals(
                doubleArrayOf(0.2, 0.15, 0.325)[axis],
                ekf.state.velocityEnuMetersPerSecond[axis],
                1.0e-12,
            )
        }
        assertEquals(22.0, ekf.state.clockBiasMeters, 1.0e-12)
        assertEquals(0.6, ekf.state.clockDriftMetersPerSecond, 1.0e-12)
        assertEquals(10_000_000_000L, ekf.state.timestampNanos)
        covarianceBefore.indices.forEach { row ->
            assertContentEquals(covarianceBefore[row], ekf.covariance[row])
        }
    }
}
