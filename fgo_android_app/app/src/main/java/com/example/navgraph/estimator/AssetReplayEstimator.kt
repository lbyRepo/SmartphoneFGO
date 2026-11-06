package com.example.navgraph.estimator

import android.content.Context

class AssetReplayEstimator(private val context: Context) : ReplayEstimator {
    override fun run(): ReplayResult {
        val started = System.nanoTime()
        val publicResult = context.assets.open(DEVICE_GNSS_ASSET).use { deviceGnss ->
            context.assets.open(GROUND_TRUTH_ASSET).use { groundTruth ->
                GsdcReplayEstimator.fromStreams(
                    deviceGnss,
                    groundTruth,
                    datasetName = "GSDC 2021 / Samsung S20 Ultra / SJC",
                    datasetScope = "600-second dynamic public segment / derived pseudorange only / not S23+ field accuracy",
                ).run()
            }
        }
        return publicResult.copy(
            gnssInsValidation = SyntheticGnssInsBenchmark().run(),
            totalComputeLatencyMillis = (System.nanoTime() - started) / 1.0e6,
        )
    }

    companion object {
        private const val DEVICE_GNSS_ASSET = "gsdc_2021_s20ultra_segment/device_gnss.csv"
        private const val GROUND_TRUTH_ASSET = "gsdc_2021_s20ultra_segment/ground_truth.csv"
    }
}
