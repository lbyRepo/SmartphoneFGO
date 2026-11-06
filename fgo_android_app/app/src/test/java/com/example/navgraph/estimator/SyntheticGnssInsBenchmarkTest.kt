package com.example.navgraph.estimator

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class SyntheticGnssInsBenchmarkTest {
    @Test
    fun ekfAndInertialFgoConvergeOnDeterministicScenario() {
        val result = SyntheticGnssInsBenchmark().run()
        println(
            "synthetic metrics: WLS=${result.wlsMetrics}, EKF=${result.ekfMetrics}, " +
                "FGO=${result.fgoMetrics}, rejectedPR=${result.ekfRejectedPseudorange}, " +
                "rejectedDoppler=${result.ekfRejectedDoppler}",
        )
        assertEquals(2_001, result.imuSampleCount)
        assertEquals(21, result.gnssEpochCount)
        assertEquals(20, result.inertialFactorCount)
        assertEquals(0, result.wlsMetrics.solverFailures)
        assertEquals(0, result.ekfMetrics.solverFailures)
        assertEquals(0, result.fgoMetrics.solverFailures)
        assertTrue(result.ekfMetrics.horizontalP50Meters < result.wlsMetrics.horizontalP50Meters)
        assertTrue(result.fgoMetrics.smoothnessRmsMeters < result.wlsMetrics.smoothnessRmsMeters)
        assertTrue(result.ekfRejectedPseudorange >= 1)
        assertTrue(result.finalPositionSigmaMeters.all { it.isFinite() && it > 0.0 })
        assertTrue(result.finalFgoPositionSigmaMeters.all { it.isFinite() && it > 0.0 })
        assertTrue(result.pseudorangeJacobianError < 2.0e-5)
        assertTrue(result.dopplerJacobianError < 2.0e-5)
    }
}
