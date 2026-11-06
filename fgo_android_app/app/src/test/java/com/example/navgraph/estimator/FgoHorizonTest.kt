package com.example.navgraph.estimator

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class FgoHorizonTest {
    @Test
    fun fiveSecondHorizonImprovesAccuracyOverTwoSeconds() {
        val dataset = SyntheticGnssInsDatasetFactory.create()
        val wls = GnssWlsSolver(applyEarthRotation = false).solve(dataset.gnssEpochs)
        val short = InertialFixedLagSmoother(
            dataset.frame,
            lagSeconds = 2.0,
            maxKeyframes = FgoConfig.MAX_KEYFRAMES,
            applyEarthRotation = false,
        ).run(dataset.gnssEpochs, dataset.imuSamples, wls)
        val configured = InertialFixedLagSmoother(
            dataset.frame,
            lagSeconds = FgoConfig.DEFAULT_LAG_SECONDS,
            maxKeyframes = FgoConfig.MAX_KEYFRAMES,
            applyEarthRotation = false,
        ).run(dataset.gnssEpochs, dataset.imuSamples, wls)
        val shortMetrics = EstimatorMetricsCalculator.compute(
            short.estimates,
            dataset.truthByTimestampMillis,
        )
        val configuredMetrics = EstimatorMetricsCalculator.compute(
            configured.estimates,
            dataset.truthByTimestampMillis,
        )
        assertEquals(0, shortMetrics.solverFailures)
        assertEquals(0, configuredMetrics.solverFailures)
        assertTrue(
            configuredMetrics.horizontalP95Meters < shortMetrics.horizontalP95Meters,
            "5 s P95 ${configuredMetrics.horizontalP95Meters} should beat 2 s P95 " +
                "${shortMetrics.horizontalP95Meters}",
        )
        assertTrue(
            configuredMetrics.horizontalP50Meters <= shortMetrics.horizontalP50Meters,
            "5 s P50 ${configuredMetrics.horizontalP50Meters} should not regress from 2 s P50 " +
                "${shortMetrics.horizontalP50Meters}",
        )
        assertTrue(configured.windowSolves == dataset.gnssEpochs.size)
        assertTrue(configured.convergedWindows == configured.windowSolves)
        assertTrue(configured.maxWindowKeyframes <= FgoConfig.MAX_KEYFRAMES)
    }

    @Test
    fun keyframeCapBoundsDenseWindowSize() {
        val dataset = SyntheticGnssInsDatasetFactory.create()
        val wls = GnssWlsSolver(applyEarthRotation = false).solve(dataset.gnssEpochs)
        val capped = InertialFixedLagSmoother(
            dataset.frame,
            lagSeconds = 30.0,
            maxKeyframes = 4,
            applyEarthRotation = false,
        ).run(dataset.gnssEpochs, dataset.imuSamples, wls)
        assertTrue(capped.maxWindowKeyframes <= 4)
        assertEquals(dataset.gnssEpochs.size, capped.estimates.size)
        assertEquals(0, EstimatorMetricsCalculator.compute(
            capped.estimates,
            dataset.truthByTimestampMillis,
        ).solverFailures)
    }

    @Test
    fun latestOnlySolvesExactlyOneWindow() {
        val dataset = SyntheticGnssInsDatasetFactory.create()
        val wls = GnssWlsSolver(applyEarthRotation = false).solve(dataset.gnssEpochs)
        val result = InertialFixedLagSmoother(
            dataset.frame,
            applyEarthRotation = false,
        ).run(dataset.gnssEpochs, dataset.imuSamples, wls, latestOnly = true)
        assertEquals(1, result.estimates.size)
        assertEquals(1, result.windowSolves)
        assertEquals(1, result.stateUncertainties.size)
        assertEquals(1, result.convergedWindows)
        assertTrue(result.maxWindowKeyframes <= FgoConfig.MAX_KEYFRAMES)
    }
}
