package com.example.navgraph.estimator

import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sqrt

internal object EstimatorMetricsCalculator {
    fun compute(
        estimates: List<PositionEstimate>,
        truthEcefByTimestampMillis: Map<Long, DoubleArray>,
    ): EstimatorMetrics {
        val horizontalErrors = mutableListOf<Double>()
        val matchedPositions = mutableListOf<DoubleArray>()
        estimates.forEach { estimate ->
            val truth = truthEcefByTimestampMillis[estimate.timestampMillis] ?: return@forEach
            val enu = Coordinates.ecefDeltaToEnu(subtract(estimate.positionEcefMeters, truth), truth)
            horizontalErrors += hypot(enu[0], enu[1])
            matchedPositions += estimate.positionEcefMeters
        }
        val jumps = matchedPositions.zipWithNext { first, second -> norm(subtract(second, first)) }
        val secondDifferences = (2 until matchedPositions.size).map { index ->
            norm(DoubleArray(3) { axis ->
                matchedPositions[index][axis] - 2.0 * matchedPositions[index - 1][axis] +
                    matchedPositions[index - 2][axis]
            })
        }
        return EstimatorMetrics(
            horizontalP50Meters = percentile(horizontalErrors, 50.0),
            horizontalP95Meters = percentile(horizontalErrors, 95.0),
            horizontalMeanMeters = horizontalErrors.average(),
            maxPositionJumpMeters = jumps.maxOrNull() ?: 0.0,
            smoothnessRmsMeters = if (secondDifferences.isEmpty()) 0.0 else
                sqrt(secondDifferences.sumOf { it * it } / secondDifferences.size),
            meanLatencyMillis = estimates.map { it.latencyMillis }.average(),
            solverFailures = estimates.count { !it.converged },
            rejectedMeasurements = estimates.sumOf { it.rejectedCount },
        )
    }

    private fun percentile(values: List<Double>, percentile: Double): Double {
        if (values.isEmpty()) return Double.NaN
        val sorted = values.sorted()
        val position = percentile / 100.0 * (sorted.size - 1)
        val lower = floor(position).toInt()
        val upper = min(lower + 1, sorted.lastIndex)
        val fraction = position - lower
        return sorted[lower] * (1.0 - fraction) + sorted[upper] * fraction
    }
}
