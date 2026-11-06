package com.example.navgraph.estimator

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

data class InertialSmootherResult(
    val estimates: List<PositionEstimate>,
    val inertialFactorCount: Int,
    val stateUncertainties: List<SmootherStateUncertainty?>,
    val engine: String = "KOTLIN",
    val nativeWindowUpdates: Int = 0,
    val nativeFailures: Int = 0,
    val windowSolves: Int = 0,
    val convergedWindows: Int = 0,
    val maxWindowKeyframes: Int = 0,
)

data class SmootherStateUncertainty(
    val positionSigmaEnuMeters: DoubleArray,
    val velocitySigmaEnuMetersPerSecond: DoubleArray,
    val clockBiasSigmaMeters: Double,
    val clockDriftSigmaMetersPerSecond: Double,
)

class InertialFixedLagSmoother(
    private val frame: EnuFrame,
    private val lagSeconds: Double = FgoConfig.DEFAULT_LAG_SECONDS,
    private val maxKeyframes: Int = FgoConfig.MAX_KEYFRAMES,
    private val applyEarthRotation: Boolean = true,
    private val preferNative: Boolean = false,
) {
    init {
        require(lagSeconds > 0.0) { "lagSeconds must be positive" }
        require(maxKeyframes >= FgoConfig.MIN_KEYFRAMES) {
            "maxKeyframes must be at least ${FgoConfig.MIN_KEYFRAMES}"
        }
    }
    fun run(
        epochs: List<GnssEpoch>,
        imuSamples: List<ImuSample>,
        wls: List<PositionEstimate>,
        latestOnly: Boolean = false,
    ): InertialSmootherResult {
        require(epochs.size == wls.size && epochs.isNotEmpty())
        val preintegrations = epochs.zipWithNext { first, second ->
            val startNanos = first.integrationTimestampNanos
            val endNanos = second.integrationTimestampNanos
            val interval = imuSamples.filter { it.timestampNanos in startNanos..endNanos }
            ImuMechanization.preintegrate(interval, UnitQuaternion.IDENTITY)
        }
        val uncertainties = mutableListOf<SmootherStateUncertainty?>()
        var nativeWindowUpdates = 0
        var nativeFailures = 0
        var engine = "KOTLIN"
        var windowSolves = 0
        var convergedWindows = 0
        var maxWindowKeyframes = 0
        val endIndices = if (latestOnly) listOf(epochs.lastIndex) else epochs.indices.toList()
        val estimates = endIndices.map { endIndex ->
            var startIndex = endIndex
            while (startIndex > 0 &&
                epochs[endIndex].integrationTimestampNanos -
                epochs[startIndex - 1].integrationTimestampNanos <=
                lagSeconds * 1.0e9 &&
                endIndex - startIndex + 2 <= maxKeyframes
            ) startIndex--
            val started = System.nanoTime()
            val solution = solveWindow(
                epochs.subList(startIndex, endIndex + 1),
                wls.subList(startIndex, endIndex + 1),
                if (endIndex == startIndex) emptyList() else
                    preintegrations.subList(startIndex, endIndex),
            )
            uncertainties += solution.uncertainty
            windowSolves++
            if (solution.converged) convergedWindows++
            maxWindowKeyframes = max(maxWindowKeyframes, endIndex - startIndex + 1)
            if (solution.engine == "C++ JNI") {
                nativeWindowUpdates++
                if (engine != "KOTLIN FALLBACK") engine = "C++ JNI"
            }
            if (solution.nativeFailure) {
                nativeFailures++
                engine = "KOTLIN FALLBACK"
            }
            val state = solution.states.last()
            val receiverPosition = frame.enuPositionToEcef(state.copyOfRange(0, 3))
            val receiverVelocity = frame.enuVectorToEcef(state.copyOfRange(3, 6))
            val rejected = epochs[endIndex].observations.count { observation ->
                val prediction = GnssGeometry.predict(
                    receiverPosition,
                    receiverVelocity,
                    state[CLOCK_BIAS_INDEX],
                    state[CLOCK_DRIFT_INDEX],
                    observation,
                    applyEarthRotation,
                )
                abs(prediction.pseudorangeMeters - observation.correctedPseudorangeMeters) /
                    observation.pseudorangeSigmaMeters > 8.0
            }
            PositionEstimate(
                timestampMillis = epochs[endIndex].timestampMillis,
                positionEcefMeters = receiverPosition,
                velocityEcefMetersPerSecond = receiverVelocity,
                clockBiasMeters = state[CLOCK_BIAS_INDEX],
                clockDriftMetersPerSecond = state[CLOCK_DRIFT_INDEX],
                iterations = solution.iterations,
                observationCount = epochs[endIndex].observations.size,
                rejectedCount = rejected,
                latencyMillis = (System.nanoTime() - started) / 1.0e6,
                converged = solution.converged,
            )
        }
        return InertialSmootherResult(
            estimates,
            preintegrations.size,
            uncertainties,
            engine,
            nativeWindowUpdates,
            nativeFailures,
            windowSolves,
            convergedWindows,
            maxWindowKeyframes,
        )
    }

    internal data class WindowSolution(
        val states: List<DoubleArray>,
        val iterations: Int,
        val converged: Boolean,
        val uncertainty: SmootherStateUncertainty?,
        val engine: String = "KOTLIN",
        val nativeFailure: Boolean = false,
    )

    internal fun solveWindow(
        epochs: List<GnssEpoch>,
        wls: List<PositionEstimate>,
        preintegrations: List<ImuPreintegration>,
    ): WindowSolution {
        var nativeFailure = false
        if (preferNative && NativeFixedLagSmoother.isAvailable) {
            val native = runCatching {
                NativeFixedLagSmoother.solveWindow(
                    frame,
                    epochs,
                    wls,
                    preintegrations,
                    applyEarthRotation,
                )
            }.getOrNull()
            if (native?.status == NativeFgoStatus.OK && native.states.size == epochs.size &&
                native.states.all { state -> state.size == STATE_SIZE && state.all(Double::isFinite) }
            ) {
                return WindowSolution(
                    states = native.states,
                    iterations = native.iterations,
                    converged = native.converged,
                    uncertainty = native.uncertainty,
                    engine = "C++ JNI",
                )
            }
            nativeFailure = true
        }
        val stateCount = epochs.size
        val dimension = stateCount * STATE_SIZE
        val states = Array(stateCount) { index -> DoubleArray(STATE_SIZE).apply {
            frame.ecefPositionToEnu(wls[index].positionEcefMeters).copyInto(this, POSITION_INDEX)
            frame.ecefVectorToEnu(wls[index].velocityEcefMetersPerSecond).copyInto(this, VELOCITY_INDEX)
            this[CLOCK_BIAS_INDEX] = wls[index].clockBiasMeters
            this[CLOCK_DRIFT_INDEX] = wls[index].clockDriftMetersPerSecond
        } }
        var converged = false
        var iterations = 0
        var lastEquations: NormalEquations? = null
        for (iteration in 1..15) {
            iterations = iteration
            val equations = NormalEquations(dimension)
            lastEquations = equations

            fun addFactor(row: DoubleArray, residual: Double, sigma: Double, robust: Boolean = false) {
                val standardized = residual / sigma
                val weight = if (robust) huberWeight(standardized) else 1.0
                equations.add(row, residual, sqrt(weight) / sigma)
            }

            val priorState = DoubleArray(STATE_SIZE).apply {
                frame.ecefPositionToEnu(wls.first().positionEcefMeters).copyInto(this, POSITION_INDEX)
                frame.ecefVectorToEnu(wls.first().velocityEcefMetersPerSecond).copyInto(this, VELOCITY_INDEX)
                this[CLOCK_BIAS_INDEX] = wls.first().clockBiasMeters
                this[CLOCK_DRIFT_INDEX] = wls.first().clockDriftMetersPerSecond
            }
            val priorSigmas = doubleArrayOf(
                8.0, 8.0, 12.0,
                2.0, 2.0, 2.0,
                0.12, 0.12, 0.12,
                100.0, 10.0,
            )
            for (component in 0 until STATE_SIZE) {
                val row = DoubleArray(dimension).apply { this[component] = 1.0 }
                addFactor(row, states[0][component] - priorState[component], priorSigmas[component])
            }

            epochs.forEachIndexed { epochIndex, epoch ->
                val state = states[epochIndex]
                val receiverPosition = frame.enuPositionToEcef(state.copyOfRange(0, 3))
                val receiverVelocity = frame.enuVectorToEcef(state.copyOfRange(3, 6))
                epoch.observations.forEach { observation ->
                    val prediction = GnssGeometry.predict(
                        receiverPosition,
                        receiverVelocity,
                        state[CLOCK_BIAS_INDEX],
                        state[CLOCK_DRIFT_INDEX],
                        observation,
                        applyEarthRotation,
                    )
                    val pseudorangeRow = DoubleArray(dimension)
                    frame.rangeDerivativeEcefToEnu(prediction.rangeDerivativeEcef).copyInto(
                        pseudorangeRow,
                        epochIndex * STATE_SIZE + POSITION_INDEX,
                    )
                    pseudorangeRow[epochIndex * STATE_SIZE + CLOCK_BIAS_INDEX] = 1.0
                    addFactor(
                        pseudorangeRow,
                        prediction.pseudorangeMeters - observation.correctedPseudorangeMeters,
                        observation.pseudorangeSigmaMeters,
                        robust = true,
                    )
                    if (observation.correctedRangeRateMetersPerSecond.isFinite() &&
                        observation.rangeRateSigmaMetersPerSecond.isFinite()
                    ) {
                        val dopplerRow = DoubleArray(dimension)
                        frame.rangeDerivativeEcefToEnu(prediction.rangeRatePositionDerivativeEcef).copyInto(
                            dopplerRow,
                            epochIndex * STATE_SIZE + POSITION_INDEX,
                        )
                        frame.rangeDerivativeEcefToEnu(prediction.rangeDerivativeEcef).copyInto(
                            dopplerRow,
                            epochIndex * STATE_SIZE + VELOCITY_INDEX,
                        )
                        dopplerRow[epochIndex * STATE_SIZE + CLOCK_DRIFT_INDEX] = 1.0
                        addFactor(
                            dopplerRow,
                            prediction.rangeRateMetersPerSecond -
                                observation.correctedRangeRateMetersPerSecond,
                            observation.rangeRateSigmaMetersPerSecond,
                            robust = true,
                        )
                    }
                }
            }

            preintegrations.forEachIndexed { intervalIndex, preintegration ->
                val firstOffset = intervalIndex * STATE_SIZE
                val secondOffset = (intervalIndex + 1) * STATE_SIZE
                val first = states[intervalIndex]
                val second = states[intervalIndex + 1]
                val dt = preintegration.deltaTimeSeconds
                for (axis in 0..2) {
                    val positionRow = DoubleArray(dimension).apply {
                        this[secondOffset + POSITION_INDEX + axis] = 1.0
                        this[firstOffset + POSITION_INDEX + axis] = -1.0
                        this[firstOffset + VELOCITY_INDEX + axis] = -dt
                        this[firstOffset + ACCEL_BIAS_INDEX + axis] = 0.5 * dt * dt
                    }
                    val positionResidual = second[POSITION_INDEX + axis] -
                        first[POSITION_INDEX + axis] - first[VELOCITY_INDEX + axis] * dt -
                        preintegration.deltaPositionEnuMeters[axis] +
                        0.5 * first[ACCEL_BIAS_INDEX + axis] * dt * dt
                    addFactor(positionRow, positionResidual, 0.20)

                    val velocityRow = DoubleArray(dimension).apply {
                        this[secondOffset + VELOCITY_INDEX + axis] = 1.0
                        this[firstOffset + VELOCITY_INDEX + axis] = -1.0
                        this[firstOffset + ACCEL_BIAS_INDEX + axis] = dt
                    }
                    val velocityResidual = second[VELOCITY_INDEX + axis] -
                        first[VELOCITY_INDEX + axis] -
                        preintegration.deltaVelocityEnuMetersPerSecond[axis] +
                        first[ACCEL_BIAS_INDEX + axis] * dt
                    addFactor(velocityRow, velocityResidual, 0.12)

                    val biasRow = DoubleArray(dimension).apply {
                        this[secondOffset + ACCEL_BIAS_INDEX + axis] = 1.0
                        this[firstOffset + ACCEL_BIAS_INDEX + axis] = -1.0
                    }
                    addFactor(
                        biasRow,
                        second[ACCEL_BIAS_INDEX + axis] - first[ACCEL_BIAS_INDEX + axis],
                        0.02,
                    )
                }
                val clockRow = DoubleArray(dimension).apply {
                    this[secondOffset + CLOCK_BIAS_INDEX] = 1.0
                    this[firstOffset + CLOCK_BIAS_INDEX] = -1.0
                    this[firstOffset + CLOCK_DRIFT_INDEX] = -dt
                }
                addFactor(
                    clockRow,
                    second[CLOCK_BIAS_INDEX] - first[CLOCK_BIAS_INDEX] -
                        first[CLOCK_DRIFT_INDEX] * dt,
                    10.0,
                )
                val driftRow = DoubleArray(dimension).apply {
                    this[secondOffset + CLOCK_DRIFT_INDEX] = 1.0
                    this[firstOffset + CLOCK_DRIFT_INDEX] = -1.0
                }
                addFactor(
                    driftRow,
                    second[CLOCK_DRIFT_INDEX] - first[CLOCK_DRIFT_INDEX],
                    1.0,
                )
            }

            val step = equations.solve(1.0e-6) ?: break
            var maximumPositionStep = 0.0
            states.forEachIndexed { stateIndex, state ->
                val offset = stateIndex * STATE_SIZE
                for (component in 0 until STATE_SIZE) state[component] += step[offset + component]
                maximumPositionStep = max(
                    maximumPositionStep,
                    norm(step.copyOfRange(offset + POSITION_INDEX, offset + POSITION_INDEX + 3)),
                )
            }
            if (maximumPositionStep < 1.0e-3) {
                converged = true
                break
            }
        }
        val lastOffset = (stateCount - 1) * STATE_SIZE
        val uncertainty = if (converged) {
            lastEquations?.inverseDiagonal(
                intArrayOf(
                    lastOffset + POSITION_INDEX,
                    lastOffset + POSITION_INDEX + 1,
                    lastOffset + POSITION_INDEX + 2,
                    lastOffset + VELOCITY_INDEX,
                    lastOffset + VELOCITY_INDEX + 1,
                    lastOffset + VELOCITY_INDEX + 2,
                    lastOffset + CLOCK_BIAS_INDEX,
                    lastOffset + CLOCK_DRIFT_INDEX,
                ),
                damping = 1.0e-6,
            )?.let { variances ->
                SmootherStateUncertainty(
                    positionSigmaEnuMeters = DoubleArray(3) { sqrt(variances[it]) },
                    velocitySigmaEnuMetersPerSecond = DoubleArray(3) { sqrt(variances[3 + it]) },
                    clockBiasSigmaMeters = sqrt(variances[6]),
                    clockDriftSigmaMetersPerSecond = sqrt(variances[7]),
                )
            }
        } else {
            null
        }
        return WindowSolution(
            states.toList(),
            iterations,
            converged,
            uncertainty,
            engine = if (nativeFailure) "KOTLIN FALLBACK" else "KOTLIN",
            nativeFailure = nativeFailure,
        )
    }

    private fun huberWeight(standardizedResidual: Double, cutoff: Double = 2.5): Double {
        val magnitude = abs(standardizedResidual)
        return if (magnitude <= cutoff) 1.0 else cutoff / magnitude
    }

    companion object {
        private const val STATE_SIZE = 11
        private const val POSITION_INDEX = 0
        private const val VELOCITY_INDEX = 3
        private const val ACCEL_BIAS_INDEX = 6
        private const val CLOCK_BIAS_INDEX = 9
        private const val CLOCK_DRIFT_INDEX = 10
    }
}
