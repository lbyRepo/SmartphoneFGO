package com.example.navgraph.live

import com.example.navgraph.estimator.Coordinates
import com.example.navgraph.estimator.EkfTranslationalVariance
import com.example.navgraph.estimator.EnuFrame
import com.example.navgraph.estimator.InsState
import com.example.navgraph.estimator.PositionEstimate
import com.example.navgraph.estimator.SmootherStateUncertainty
import com.example.navgraph.estimator.UnitQuaternion
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class FgoFeedbackGateTest {
    private val frame = EnuFrame(Coordinates.llaToEcef(1.3521, 103.8198, 35.0))
    private val reference = FgoFeedbackReference(
        elapsedRealtimeNanos = 10_000_000_000L,
        state = InsState(
            timestampNanos = 10_000_000_000L,
            positionEnuMeters = DoubleArray(3),
            velocityEnuMetersPerSecond = DoubleArray(3),
            attitudeBodyToEnu = UnitQuaternion.IDENTITY,
            accelerometerBiasBodyMetersPerSecondSquared = DoubleArray(3),
            gyroBiasBodyRadiansPerSecond = DoubleArray(3),
            clockBiasMeters = 20.0,
            clockDriftMetersPerSecond = 0.5,
        ),
        variance = EkfTranslationalVariance(
            positionEnuMetersSquared = doubleArrayOf(4.0, 4.0, 9.0),
            velocityEnuMetersPerSecondSquared = doubleArrayOf(0.25, 0.25, 0.25),
            clockBiasMetersSquared = 100.0,
            clockDriftMetersPerSecondSquared = 1.0,
        ),
    )
    private val uncertainty = SmootherStateUncertainty(
        positionSigmaEnuMeters = doubleArrayOf(1.0, 1.0, 1.5),
        velocitySigmaEnuMetersPerSecond = doubleArrayOf(0.2, 0.2, 0.2),
        clockBiasSigmaMeters = 5.0,
        clockDriftSigmaMetersPerSecond = 0.5,
    )

    @Test
    fun acceptsBoundedRecentCovarianceBackedCorrection() {
        val decision = FgoFeedbackGate.evaluate(
            reference,
            estimate(positionEnu = doubleArrayOf(2.0, -1.0, 0.5)),
            uncertainty,
            frame,
            currentElapsedRealtimeNanos = 10_500_000_000L,
        )
        assertTrue(decision.accepted, decision.reason)
        assertEquals("accepted", decision.reason)
        assertTrue(decision.normalizedInnovationSquared in 0.0..22.46)
    }

    @Test
    fun rejectsStaleAndUnboundedCorrectionsWithStableReasons() {
        val stale = FgoFeedbackGate.evaluate(
            reference,
            estimate(doubleArrayOf(1.0, 0.0, 0.0)),
            uncertainty,
            frame,
            currentElapsedRealtimeNanos = 12_000_000_001L,
        )
        assertFalse(stale.accepted)
        assertEquals("stale_solution", stale.reason)

        val unbounded = FgoFeedbackGate.evaluate(
            reference,
            estimate(doubleArrayOf(21.0, 0.0, 0.0)),
            uncertainty,
            frame,
            currentElapsedRealtimeNanos = reference.elapsedRealtimeNanos,
        )
        assertFalse(unbounded.accepted)
        assertEquals("position_bound", unbounded.reason)
    }

    @Test
    fun rejectsMissingCovariance() {
        val decision = FgoFeedbackGate.evaluate(
            reference,
            estimate(doubleArrayOf(1.0, 0.0, 0.0)),
            uncertainty = null,
            frame,
            currentElapsedRealtimeNanos = reference.elapsedRealtimeNanos,
        )
        assertFalse(decision.accepted)
        assertEquals("covariance_unavailable", decision.reason)
    }

    private fun estimate(positionEnu: DoubleArray): PositionEstimate = PositionEstimate(
        timestampMillis = 10_000L,
        positionEcefMeters = frame.enuPositionToEcef(positionEnu),
        velocityEcefMetersPerSecond = frame.enuVectorToEcef(doubleArrayOf(0.2, -0.1, 0.0)),
        clockBiasMeters = 22.0,
        clockDriftMetersPerSecond = 0.6,
        iterations = 3,
        observationCount = 8,
        rejectedCount = 0,
        latencyMillis = 1.0,
        converged = true,
    )
}
