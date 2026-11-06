package com.example.navgraph.estimator

data class NativeWlsResult(
    val estimate: PositionEstimate,
    val numericallyValid: Boolean,
)

/** Optional JNI WLS path. Kotlin remains authoritative when the native library is absent. */
object NativeGnssWls {
    private const val OBSERVATION_STRIDE = 10
    private const val RESULT_SIZE = 13

    @Volatile
    private var loaded = runCatching { System.loadLibrary("navgraph_estimator") }.isSuccess

    val isAvailable: Boolean
        get() = loaded

    val readinessLabel: String
        get() = if (loaded) "C++ JNI READY" else "KOTLIN"

    fun solveEpoch(
        epoch: GnssEpoch,
        initial: PositionEstimate? = null,
        applyEarthRotation: Boolean = true,
    ): NativeWlsResult? {
        if (!loaded) return null
        require(epoch.observations.size >= 4)
        val packed = DoubleArray(epoch.observations.size * OBSERVATION_STRIDE)
        epoch.observations.forEachIndexed { index, observation ->
            val offset = index * OBSERVATION_STRIDE
            observation.satellitePositionMeters.copyInto(packed, offset)
            observation.satelliteVelocityMetersPerSecond.copyInto(packed, offset + 3)
            packed[offset + 6] = observation.correctedPseudorangeMeters
            packed[offset + 7] = observation.pseudorangeSigmaMeters
            packed[offset + 8] = observation.correctedRangeRateMetersPerSecond
            packed[offset + 9] = observation.rangeRateSigmaMetersPerSecond
        }
        val seed = DoubleArray(8)
        initial?.let { estimate ->
            estimate.positionEcefMeters.copyInto(seed, 0)
            estimate.velocityEcefMetersPerSecond.copyInto(seed, 3)
            seed[6] = estimate.clockBiasMeters
            seed[7] = estimate.clockDriftMetersPerSecond
        }
        val started = System.nanoTime()
        val values = nativeSolve(
            epoch.timestampMillis,
            epoch.integrationTimestampNanos,
            packed,
            seed,
            applyEarthRotation,
        )
        require(values.size == RESULT_SIZE) { "Native WLS returned ${values.size} values" }
        return NativeWlsResult(
            estimate = PositionEstimate(
                timestampMillis = epoch.timestampMillis,
                positionEcefMeters = values.copyOfRange(0, 3),
                velocityEcefMetersPerSecond = values.copyOfRange(3, 6),
                clockBiasMeters = values[6],
                clockDriftMetersPerSecond = values[7],
                iterations = values[8].toInt(),
                observationCount = values[9].toInt(),
                rejectedCount = values[10].toInt(),
                latencyMillis = (System.nanoTime() - started) / 1.0e6,
                converged = values[11] == 1.0,
            ),
            numericallyValid = values[12] == 1.0,
        )
    }

    internal fun loadAbsoluteForTests(path: String) {
        System.load(path)
        loaded = true
    }

    private external fun nativeSolve(
        timestampMillis: Long,
        integrationTimestampNanos: Long,
        packedObservations: DoubleArray,
        initialState: DoubleArray,
        applyEarthRotation: Boolean,
    ): DoubleArray
}
