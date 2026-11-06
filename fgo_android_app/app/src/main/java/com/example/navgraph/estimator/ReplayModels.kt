package com.example.navgraph.estimator

data class GnssObservation(
    val timestampMillis: Long,
    val constellation: Int,
    val svid: Int,
    val signalType: String,
    val satellitePositionMeters: DoubleArray,
    val satelliteVelocityMetersPerSecond: DoubleArray,
    val correctedPseudorangeMeters: Double,
    val pseudorangeSigmaMeters: Double,
    val correctedRangeRateMetersPerSecond: Double,
    val rangeRateSigmaMetersPerSecond: Double,
    val cn0DbHz: Double,
    val elevationDegrees: Double,
)

data class GnssEpoch(
    val timestampMillis: Long,
    val observations: List<GnssObservation>,
    val elapsedRealtimeNanos: Long? = null,
) {
    /** Monotonic integration time live; replay time only when no Android source time exists. */
    val integrationTimestampNanos: Long
        get() = elapsedRealtimeNanos ?: Math.multiplyExact(timestampMillis, 1_000_000L)
}

data class PositionEstimate(
    val timestampMillis: Long,
    val positionEcefMeters: DoubleArray,
    val velocityEcefMetersPerSecond: DoubleArray,
    val clockBiasMeters: Double,
    val clockDriftMetersPerSecond: Double,
    val iterations: Int,
    val observationCount: Int,
    val rejectedCount: Int,
    val latencyMillis: Double,
    val converged: Boolean,
)

data class HorizontalPoint(val eastMeters: Double, val northMeters: Double)

data class EstimatorMetrics(
    val horizontalP50Meters: Double,
    val horizontalP95Meters: Double,
    val horizontalMeanMeters: Double,
    val maxPositionJumpMeters: Double,
    val smoothnessRmsMeters: Double,
    val meanLatencyMillis: Double,
    val solverFailures: Int,
    val rejectedMeasurements: Int,
)

data class ReplayResult(
    val datasetName: String,
    val epochCount: Int,
    val acceptedObservationCount: Int,
    val preprocessingRejections: Map<String, Int>,
    val wlsMetrics: EstimatorMetrics,
    val fgoMetrics: EstimatorMetrics,
    val truthTrajectory: List<HorizontalPoint>,
    val wlsTrajectory: List<HorizontalPoint>,
    val fgoTrajectory: List<HorizontalPoint>,
    val pseudorangeJacobianError: Double,
    val dopplerJacobianError: Double,
    val datasetScope: String = "",
    val measurementProfile: String = "",
    val gnssInsValidation: GnssInsValidation? = null,
    val totalComputeLatencyMillis: Double = 0.0,
)

data class GnssInsValidation(
    val durationSeconds: Double,
    val imuSampleCount: Int,
    val gnssEpochCount: Int,
    val inertialFactorCount: Int,
    val wlsMetrics: EstimatorMetrics,
    val ekfMetrics: EstimatorMetrics,
    val fgoMetrics: EstimatorMetrics,
    val truthTrajectory: List<HorizontalPoint>,
    val wlsTrajectory: List<HorizontalPoint>,
    val ekfTrajectory: List<HorizontalPoint>,
    val fgoTrajectory: List<HorizontalPoint>,
    val ekfAcceptedPseudorange: Int,
    val ekfRejectedPseudorange: Int,
    val ekfAcceptedDoppler: Int,
    val ekfRejectedDoppler: Int,
    val finalPositionSigmaMeters: DoubleArray,
    val finalFgoPositionSigmaMeters: DoubleArray,
    val pseudorangeJacobianError: Double,
    val dopplerJacobianError: Double,
)

interface ReplayEstimator {
    fun run(): ReplayResult
}
