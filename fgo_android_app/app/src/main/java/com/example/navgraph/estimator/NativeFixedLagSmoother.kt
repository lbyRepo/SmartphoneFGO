package com.example.navgraph.estimator

enum class NativeFgoStatus {
    OK,
    INVALID_INPUT,
    NON_FINITE_INPUT,
    INVALID_FRAME,
    LINEAR_SOLVE_FAILURE,
}

data class NativeFgoWindowResult(
    val states: List<DoubleArray>,
    val iterations: Int,
    val converged: Boolean,
    val status: NativeFgoStatus,
    val skippedFactors: Int,
    val uncertainty: SmootherStateUncertainty?,
)

object NativeFixedLagSmoother {
    private const val FRAME_SIZE = 12
    private const val STATE_SIZE = 11
    private const val OBSERVATION_STRIDE = 10
    private const val WLS_STRIDE = 8
    private const val PREINTEGRATION_STRIDE = 7
    private const val RESULT_TAIL_SIZE = 13

    @Volatile
    private var loaded = runCatching { System.loadLibrary("navgraph_estimator") }.isSuccess

    val isAvailable: Boolean
        get() = loaded

    val readinessLabel: String
        get() = if (loaded) "C++ JNI READY" else "KOTLIN"

    fun solveWindow(
        frame: EnuFrame,
        epochs: List<GnssEpoch>,
        wls: List<PositionEstimate>,
        preintegrations: List<ImuPreintegration>,
        applyEarthRotation: Boolean = true,
    ): NativeFgoWindowResult? {
        if (!loaded) return null
        require(epochs.isNotEmpty() && epochs.size == wls.size)
        require(preintegrations.size + 1 == epochs.size)
        val packedFrame = DoubleArray(FRAME_SIZE)
        frame.referenceEcefMeters.copyInto(packedFrame, 0)
        frame.ecefToEnuRowMajor().copyInto(packedFrame, 3)
        val offsets = IntArray(epochs.size + 1)
        epochs.indices.forEach { index ->
            offsets[index + 1] = offsets[index] + epochs[index].observations.size
        }
        val observations = DoubleArray(offsets.last() * OBSERVATION_STRIDE)
        epochs.forEachIndexed { epochIndex, epoch ->
            epoch.observations.forEachIndexed { observationIndex, observation ->
                val offset = (offsets[epochIndex] + observationIndex) * OBSERVATION_STRIDE
                observation.satellitePositionMeters.copyInto(observations, offset)
                observation.satelliteVelocityMetersPerSecond.copyInto(observations, offset + 3)
                observations[offset + 6] = observation.correctedPseudorangeMeters
                observations[offset + 7] = observation.pseudorangeSigmaMeters
                observations[offset + 8] = observation.correctedRangeRateMetersPerSecond
                observations[offset + 9] = observation.rangeRateSigmaMetersPerSecond
            }
        }
        val packedWls = DoubleArray(wls.size * WLS_STRIDE)
        wls.forEachIndexed { index, estimate ->
            val offset = index * WLS_STRIDE
            estimate.positionEcefMeters.copyInto(packedWls, offset)
            estimate.velocityEcefMetersPerSecond.copyInto(packedWls, offset + 3)
            packedWls[offset + 6] = estimate.clockBiasMeters
            packedWls[offset + 7] = estimate.clockDriftMetersPerSecond
        }
        val packedPreintegrations = DoubleArray(preintegrations.size * PREINTEGRATION_STRIDE)
        preintegrations.forEachIndexed { index, preintegration ->
            val offset = index * PREINTEGRATION_STRIDE
            packedPreintegrations[offset] = preintegration.deltaTimeSeconds
            preintegration.deltaPositionEnuMeters.copyInto(packedPreintegrations, offset + 1)
            preintegration.deltaVelocityEnuMetersPerSecond.copyInto(
                packedPreintegrations,
                offset + 4,
            )
        }
        val result = nativeSolveWindow(
            packedFrame,
            LongArray(epochs.size) { epochs[it].timestampMillis },
            LongArray(epochs.size) { epochs[it].integrationTimestampNanos },
            offsets,
            observations,
            packedWls,
            packedPreintegrations,
            applyEarthRotation,
        )
        require(result.size >= RESULT_TAIL_SIZE &&
            (result.size - RESULT_TAIL_SIZE) % STATE_SIZE == 0
        ) { "Native FGO returned ${result.size} values" }
        val stateCount = (result.size - RESULT_TAIL_SIZE) / STATE_SIZE
        val tail = stateCount * STATE_SIZE
        val statusIndex = result[tail + 2].toInt()
        require(statusIndex in NativeFgoStatus.entries.indices) {
            "Native FGO returned invalid status $statusIndex"
        }
        val uncertainty = if (result[tail + 4] == 1.0) {
            SmootherStateUncertainty(
                positionSigmaEnuMeters = result.copyOfRange(tail + 5, tail + 8),
                velocitySigmaEnuMetersPerSecond = result.copyOfRange(tail + 8, tail + 11),
                clockBiasSigmaMeters = result[tail + 11],
                clockDriftSigmaMetersPerSecond = result[tail + 12],
            )
        } else {
            null
        }
        return NativeFgoWindowResult(
            states = List(stateCount) { index ->
                result.copyOfRange(index * STATE_SIZE, (index + 1) * STATE_SIZE)
            },
            iterations = result[tail].toInt(),
            converged = result[tail + 1] == 1.0,
            status = NativeFgoStatus.entries[statusIndex],
            skippedFactors = result[tail + 3].toInt(),
            uncertainty = uncertainty,
        )
    }

    internal fun loadAbsoluteForTests(path: String) {
        System.load(path)
        loaded = true
    }

    private external fun nativeSolveWindow(
        packedFrame: DoubleArray,
        epochTimestampsMillis: LongArray,
        epochIntegrationTimestampsNanos: LongArray,
        observationOffsets: IntArray,
        packedObservations: DoubleArray,
        packedWls: DoubleArray,
        packedPreintegrations: DoubleArray,
        applyEarthRotation: Boolean,
    ): DoubleArray
}
