package com.example.navgraph.estimator

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

class GnssWlsSolver(private val applyEarthRotation: Boolean = true) {
    fun solve(epochs: List<GnssEpoch>): List<PositionEstimate> {
        val estimates = mutableListOf<PositionEstimate>()
        val positionClock = DoubleArray(4)
        var velocityClock = DoubleArray(4)
        epochs.forEach { epoch ->
            val estimate = solveEpoch(
                epoch,
                positionClock.copyOfRange(0, 3),
                velocityClock.copyOfRange(0, 3),
                positionClock[3],
                velocityClock[3],
            )
            estimate.positionEcefMeters.copyInto(positionClock, 0)
            positionClock[3] = estimate.clockBiasMeters
            velocityClock = doubleArrayOf(
                *estimate.velocityEcefMetersPerSecond,
                estimate.clockDriftMetersPerSecond,
            )
            estimates += estimate
        }
        return estimates
    }

    fun solveEpoch(epoch: GnssEpoch, initial: PositionEstimate? = null): PositionEstimate = solveEpoch(
        epoch = epoch,
        initialPositionEcefMeters = initial?.positionEcefMeters ?: DoubleArray(3),
        initialVelocityEcefMetersPerSecond = initial?.velocityEcefMetersPerSecond ?: DoubleArray(3),
        initialClockBiasMeters = initial?.clockBiasMeters ?: 0.0,
        initialClockDriftMetersPerSecond = initial?.clockDriftMetersPerSecond ?: 0.0,
    )

    private fun solveEpoch(
        epoch: GnssEpoch,
        initialPositionEcefMeters: DoubleArray,
        initialVelocityEcefMetersPerSecond: DoubleArray,
        initialClockBiasMeters: Double,
        initialClockDriftMetersPerSecond: Double,
    ): PositionEstimate {
        val started = System.nanoTime()
        val positionClock = doubleArrayOf(
            *initialPositionEcefMeters,
            initialClockBiasMeters,
        )
        var velocityClock = doubleArrayOf(
            *initialVelocityEcefMetersPerSecond,
            initialClockDriftMetersPerSecond,
        )
        var converged = false
        var iterations = 0
        for (iteration in 1..20) {
            iterations = iteration
            val equations = NormalEquations(4)
            epoch.observations.forEach { observation ->
                val prediction = GnssGeometry.predict(
                    positionClock.copyOfRange(0, 3),
                    velocityClock.copyOfRange(0, 3),
                    positionClock[3],
                    velocityClock[3],
                    observation,
                    applyEarthRotation,
                )
                val residual = prediction.pseudorangeMeters - observation.correctedPseudorangeMeters
                val row = doubleArrayOf(
                    prediction.rangeDerivativeEcef[0],
                    prediction.rangeDerivativeEcef[1],
                    prediction.rangeDerivativeEcef[2],
                    1.0,
                )
                val scale = sqrt(huberWeight(residual / observation.pseudorangeSigmaMeters)) /
                    observation.pseudorangeSigmaMeters
                equations.add(row, residual, scale)
            }
            val step = equations.solve(1.0e-9) ?: break
            for (index in positionClock.indices) positionClock[index] += step[index]
            if (norm(step.copyOfRange(0, 3)) < 1.0e-3 && abs(step[3]) < 1.0e-3) {
                converged = true
                break
            }
        }
        velocityClock = solveVelocity(epoch, positionClock, velocityClock)
        val rejected = epoch.observations.count { observation ->
            val prediction = GnssGeometry.predict(
                positionClock.copyOfRange(0, 3),
                velocityClock.copyOfRange(0, 3),
                positionClock[3],
                velocityClock[3],
                observation,
                applyEarthRotation,
            )
            abs(prediction.pseudorangeMeters - observation.correctedPseudorangeMeters) /
                observation.pseudorangeSigmaMeters > 8.0
        }
        return PositionEstimate(
            timestampMillis = epoch.timestampMillis,
            positionEcefMeters = positionClock.copyOfRange(0, 3),
            velocityEcefMetersPerSecond = velocityClock.copyOfRange(0, 3),
            clockBiasMeters = positionClock[3],
            clockDriftMetersPerSecond = velocityClock[3],
            iterations = iterations,
            observationCount = epoch.observations.size,
            rejectedCount = rejected,
            latencyMillis = (System.nanoTime() - started) / 1.0e6,
            converged = converged,
        )
    }

    private fun solveVelocity(
        epoch: GnssEpoch,
        positionClock: DoubleArray,
        initial: DoubleArray,
    ): DoubleArray {
        val state = initial.copyOf()
        repeat(3) {
            val equations = NormalEquations(4)
            var factorCount = 0
            epoch.observations.forEach { observation ->
                if (!observation.correctedRangeRateMetersPerSecond.isFinite() ||
                    !observation.rangeRateSigmaMetersPerSecond.isFinite()
                ) return@forEach
                val prediction = GnssGeometry.predict(
                    positionClock.copyOfRange(0, 3),
                    state.copyOfRange(0, 3),
                    positionClock[3],
                    state[3],
                    observation,
                    applyEarthRotation,
                )
                val residual = prediction.rangeRateMetersPerSecond -
                    observation.correctedRangeRateMetersPerSecond
                val row = doubleArrayOf(
                    prediction.rangeDerivativeEcef[0],
                    prediction.rangeDerivativeEcef[1],
                    prediction.rangeDerivativeEcef[2],
                    1.0,
                )
                val sigma = max(observation.rangeRateSigmaMetersPerSecond, 0.15)
                equations.add(row, residual, sqrt(huberWeight(residual / sigma)) / sigma)
                factorCount++
            }
            if (factorCount < 4) return state
            val step = equations.solve(1.0e-9) ?: return state
            for (index in state.indices) state[index] += step[index]
            if (norm(step) < 1.0e-5) return state
        }
        return state
    }

    private fun huberWeight(standardizedResidual: Double, cutoff: Double = 2.5): Double {
        val magnitude = abs(standardizedResidual)
        return if (magnitude <= cutoff) 1.0 else cutoff / magnitude
    }
}
