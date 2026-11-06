package com.example.navgraph.estimator

import java.io.File
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class GsdcReplayEstimatorTest {
    @Test
    fun bundledFixtureConvergesWithCheckedJacobians() {
        val assetRoot = File("src/main/assets/gsdc_2023_sample")
        val result = File(assetRoot, "device_gnss.csv").inputStream().use { device ->
            File(assetRoot, "ground_truth.csv").inputStream().use { truth ->
                GsdcReplayEstimator.fromStreams(device, truth).run()
            }
        }

        assertEquals(5, result.epochCount)
        assertEquals(169, result.acceptedObservationCount)
        assertEquals(0, result.wlsMetrics.solverFailures)
        assertEquals(0, result.fgoMetrics.solverFailures)
        assertTrue(result.pseudorangeJacobianError < 2.0e-5)
        assertTrue(result.dopplerJacobianError < 2.0e-5)
        assertTrue(result.wlsMetrics.horizontalP50Meters in 2.5..3.1)
        assertTrue(result.fgoMetrics.horizontalP50Meters in 2.5..3.2)
        assertTrue(result.fgoMetrics.maxPositionJumpMeters < result.wlsMetrics.maxPositionJumpMeters)
        assertTrue(result.fgoMetrics.smoothnessRmsMeters < result.wlsMetrics.smoothnessRmsMeters)
    }

    @Test
    fun samsungS20SegmentReplaysWithLegacyDerivedSchema() {
        val assetRoot = File("src/main/assets/gsdc_2021_s20ultra_segment")
        val result = File(assetRoot, "device_gnss.csv").inputStream().use { device ->
            File(assetRoot, "ground_truth.csv").inputStream().use { truth ->
                GsdcReplayEstimator.fromStreams(
                    device,
                    truth,
                    datasetName = "GSDC 2021 / Samsung S20 Ultra / SJC",
                ).run()
            }
        }

        assertEquals(600, result.epochCount)
        assertEquals(15_554, result.acceptedObservationCount)
        assertEquals(1, result.wlsMetrics.solverFailures)
        assertEquals(3, result.fgoMetrics.solverFailures)
        assertTrue(result.dopplerJacobianError.isNaN())
        assertTrue(result.pseudorangeJacobianError < 2.0e-5)
        assertTrue(result.wlsMetrics.horizontalP50Meters in 8.7..9.1)
        assertTrue(result.fgoMetrics.horizontalP50Meters in 8.8..9.2)
        assertTrue(result.fgoMetrics.horizontalP95Meters < result.wlsMetrics.horizontalP95Meters)
        assertTrue(result.fgoMetrics.maxPositionJumpMeters < result.wlsMetrics.maxPositionJumpMeters)
        assertTrue(result.fgoMetrics.smoothnessRmsMeters < result.wlsMetrics.smoothnessRmsMeters)
    }
}
