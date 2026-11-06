package com.example.navgraph.estimator

import kotlin.math.abs
import kotlin.math.max

object GnssInsJacobianChecker {
    fun check(
        frame: EnuFrame,
        state: InsState,
        observation: GnssObservation,
        applyEarthRotation: Boolean,
    ): Pair<Double, Double> {
        val compactState = DoubleArray(8).apply {
            state.positionEnuMeters.copyInto(this, 0)
            state.velocityEnuMetersPerSecond.copyInto(this, 3)
            this[6] = state.clockBiasMeters
            this[7] = state.clockDriftMetersPerSecond
        }
        val prediction = prediction(frame, compactState, observation, applyEarthRotation)
        val pseudorangeAnalytic = DoubleArray(8).apply {
            frame.rangeDerivativeEcefToEnu(prediction.rangeDerivativeEcef).copyInto(this, 0)
            this[6] = 1.0
        }
        val dopplerAnalytic = DoubleArray(8).apply {
            frame.rangeDerivativeEcefToEnu(prediction.rangeRatePositionDerivativeEcef).copyInto(this, 0)
            frame.rangeDerivativeEcefToEnu(prediction.rangeDerivativeEcef).copyInto(this, 3)
            this[7] = 1.0
        }
        return maximumError(
            compactState,
            pseudorangeAnalytic,
        ) { candidate -> prediction(frame, candidate, observation, applyEarthRotation).pseudorangeMeters } to
            maximumError(
                compactState,
                dopplerAnalytic,
            ) { candidate -> prediction(frame, candidate, observation, applyEarthRotation).rangeRateMetersPerSecond }
    }

    private fun prediction(
        frame: EnuFrame,
        state: DoubleArray,
        observation: GnssObservation,
        applyEarthRotation: Boolean,
    ): GnssPrediction = GnssGeometry.predict(
        frame.enuPositionToEcef(state.copyOfRange(0, 3)),
        frame.enuVectorToEcef(state.copyOfRange(3, 6)),
        state[6],
        state[7],
        observation,
        applyEarthRotation,
    )

    private fun maximumError(
        state: DoubleArray,
        analytic: DoubleArray,
        value: (DoubleArray) -> Double,
    ): Double {
        var maximum = 0.0
        state.indices.forEach { index ->
            val epsilon = if (index == 7) 1.0e-4 else 1.0e-3
            val plus = state.copyOf().apply { this[index] += epsilon }
            val minus = state.copyOf().apply { this[index] -= epsilon }
            val numerical = (value(plus) - value(minus)) / (2.0 * epsilon)
            maximum = max(maximum, abs(analytic[index] - numerical))
        }
        return maximum
    }
}
