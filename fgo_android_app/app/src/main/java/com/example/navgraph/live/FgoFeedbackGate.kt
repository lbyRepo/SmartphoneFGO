package com.example.navgraph.live

import com.example.navgraph.estimator.EkfTranslationalVariance
import com.example.navgraph.estimator.EnuFrame
import com.example.navgraph.estimator.InsState
import com.example.navgraph.estimator.PositionEstimate
import com.example.navgraph.estimator.SmootherStateUncertainty
import kotlin.math.abs
import kotlin.math.sqrt

internal data class FgoFeedbackReference(
    val elapsedRealtimeNanos: Long,
    val state: InsState,
    val variance: EkfTranslationalVariance,
)

internal data class FgoFeedbackDecision(
    val accepted: Boolean,
    val reason: String,
    val positionDeltaEnuMeters: DoubleArray = DoubleArray(3),
    val velocityDeltaEnuMetersPerSecond: DoubleArray = DoubleArray(3),
    val clockBiasDeltaMeters: Double = 0.0,
    val clockDriftDeltaMetersPerSecond: Double = 0.0,
    val normalizedInnovationSquared: Double = Double.NaN,
) {
    val positionCorrectionNormMeters: Double
        get() = sqrt(positionDeltaEnuMeters.sumOf { it * it })
}

internal object FgoFeedbackGate {
    fun evaluate(
        reference: FgoFeedbackReference,
        estimate: PositionEstimate,
        uncertainty: SmootherStateUncertainty?,
        frame: EnuFrame,
        currentElapsedRealtimeNanos: Long,
    ): FgoFeedbackDecision {
        if (!estimate.converged) return rejected("not_converged")
        if (uncertainty == null) return rejected("covariance_unavailable")
        val age = currentElapsedRealtimeNanos - reference.elapsedRealtimeNanos
        if (age < 0L) return rejected("future_solution")
        if (age > MAX_SOLUTION_AGE_NANOS) return rejected("stale_solution")
        if (!uncertainty.isUsable() || !reference.variance.isUsable()) {
            return rejected("invalid_covariance")
        }

        val targetPosition = frame.ecefPositionToEnu(estimate.positionEcefMeters)
        val targetVelocity = frame.ecefVectorToEnu(estimate.velocityEcefMetersPerSecond)
        val positionDelta = DoubleArray(3) { targetPosition[it] - reference.state.positionEnuMeters[it] }
        val velocityDelta = DoubleArray(3) {
            targetVelocity[it] - reference.state.velocityEnuMetersPerSecond[it]
        }
        val clockBiasDelta = estimate.clockBiasMeters - reference.state.clockBiasMeters
        val clockDriftDelta = estimate.clockDriftMetersPerSecond -
            reference.state.clockDriftMetersPerSecond
        if (!(positionDelta + velocityDelta).all(Double::isFinite) ||
            !clockBiasDelta.isFinite() || !clockDriftDelta.isFinite()
        ) return rejected("non_finite_correction")

        val positionNorm = norm(positionDelta)
        val velocityNorm = norm(velocityDelta)
        if (positionNorm > MAX_POSITION_DELTA_METERS) return rejected("position_bound")
        if (velocityNorm > MAX_VELOCITY_DELTA_METERS_PER_SECOND) return rejected("velocity_bound")
        if (abs(clockBiasDelta) > MAX_CLOCK_BIAS_DELTA_METERS) return rejected("clock_bias_bound")
        if (abs(clockDriftDelta) > MAX_CLOCK_DRIFT_DELTA_METERS_PER_SECOND) {
            return rejected("clock_drift_bound")
        }

        var normalizedInnovationSquared = 0.0
        for (axis in 0..2) {
            val positionVariance = CORRELATION_INFLATION * (
                reference.variance.positionEnuMetersSquared[axis] +
                    uncertainty.positionSigmaEnuMeters[axis].squared()
                )
            val velocityVariance = CORRELATION_INFLATION * (
                reference.variance.velocityEnuMetersPerSecondSquared[axis] +
                    uncertainty.velocitySigmaEnuMetersPerSecond[axis].squared()
                )
            normalizedInnovationSquared += positionDelta[axis].squared() / positionVariance
            normalizedInnovationSquared += velocityDelta[axis].squared() / velocityVariance
        }
        if (!normalizedInnovationSquared.isFinite() || normalizedInnovationSquared > NIS_GATE) {
            return rejected("normalized_innovation_gate", normalizedInnovationSquared)
        }
        return FgoFeedbackDecision(
            accepted = true,
            reason = "accepted",
            positionDeltaEnuMeters = positionDelta,
            velocityDeltaEnuMetersPerSecond = velocityDelta,
            clockBiasDeltaMeters = clockBiasDelta,
            clockDriftDeltaMetersPerSecond = clockDriftDelta,
            normalizedInnovationSquared = normalizedInnovationSquared,
        )
    }

    private fun rejected(reason: String, nis: Double = Double.NaN) = FgoFeedbackDecision(
        accepted = false,
        reason = reason,
        normalizedInnovationSquared = nis,
    )

    private fun SmootherStateUncertainty.isUsable(): Boolean =
        positionSigmaEnuMeters.size == 3 && velocitySigmaEnuMetersPerSecond.size == 3 &&
            positionSigmaEnuMeters.all { it.isFinite() && it > 0.0 } &&
            velocitySigmaEnuMetersPerSecond.all { it.isFinite() && it > 0.0 } &&
            clockBiasSigmaMeters.isFinite() && clockBiasSigmaMeters > 0.0 &&
            clockDriftSigmaMetersPerSecond.isFinite() && clockDriftSigmaMetersPerSecond > 0.0

    private fun EkfTranslationalVariance.isUsable(): Boolean =
        positionEnuMetersSquared.size == 3 && velocityEnuMetersPerSecondSquared.size == 3 &&
            positionEnuMetersSquared.all { it.isFinite() && it > 0.0 } &&
            velocityEnuMetersPerSecondSquared.all { it.isFinite() && it > 0.0 } &&
            clockBiasMetersSquared.isFinite() && clockBiasMetersSquared > 0.0 &&
            clockDriftMetersPerSecondSquared.isFinite() && clockDriftMetersPerSecondSquared > 0.0

    private fun norm(values: DoubleArray): Double = sqrt(values.sumOf { it * it })
    private fun Double.squared(): Double = this * this

    const val FEEDBACK_GAIN = 0.25
    private const val CORRELATION_INFLATION = 4.0
    private const val NIS_GATE = 22.46
    private const val MAX_SOLUTION_AGE_NANOS = 2_000_000_000L
    private const val MAX_POSITION_DELTA_METERS = 20.0
    private const val MAX_VELOCITY_DELTA_METERS_PER_SECOND = 5.0
    private const val MAX_CLOCK_BIAS_DELTA_METERS = 100.0
    private const val MAX_CLOCK_DRIFT_DELTA_METERS_PER_SECOND = 10.0
}
