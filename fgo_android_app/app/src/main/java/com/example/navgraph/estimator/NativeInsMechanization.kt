package com.example.navgraph.estimator

enum class NativeInsStatus {
    OK,
    NON_INCREASING_TIMESTAMP,
    STATE_TIMESTAMP_MISMATCH,
    NON_FINITE_INPUT,
    INVALID_QUATERNION,
}

data class NativeInsPropagationResult(
    val state: InsState,
    val status: NativeInsStatus,
)

object NativeInsMechanization {
    private const val STATE_SIZE = 18
    private const val SAMPLE_SIZE = 6
    private const val RESULT_SIZE = 19

    @Volatile
    private var loaded = runCatching { System.loadLibrary("navgraph_estimator") }.isSuccess

    val isAvailable: Boolean
        get() = loaded

    val readinessLabel: String
        get() = if (loaded) "C++ JNI READY" else "KOTLIN"

    fun propagate(
        state: InsState,
        previous: ImuSample,
        current: ImuSample,
    ): NativeInsPropagationResult? {
        if (!loaded) return null
        val values = nativePropagate(
            state.timestampNanos,
            state.pack(),
            previous.timestampNanos,
            previous.pack(),
            current.timestampNanos,
            current.pack(),
        )
        require(values.size == RESULT_SIZE) {
            "Native INS returned ${values.size} values"
        }
        val statusIndex = values[18].toInt()
        require(statusIndex in NativeInsStatus.entries.indices) {
            "Native INS returned invalid status $statusIndex"
        }
        return NativeInsPropagationResult(
            state = InsState(
                timestampNanos = if (statusIndex == NativeInsStatus.OK.ordinal) {
                    current.timestampNanos
                } else {
                    state.timestampNanos
                },
                positionEnuMeters = values.copyOfRange(0, 3),
                velocityEnuMetersPerSecond = values.copyOfRange(3, 6),
                attitudeBodyToEnu = UnitQuaternion(values[6], values[7], values[8], values[9]),
                accelerometerBiasBodyMetersPerSecondSquared = values.copyOfRange(10, 13),
                gyroBiasBodyRadiansPerSecond = values.copyOfRange(13, 16),
                clockBiasMeters = values[16],
                clockDriftMetersPerSecond = values[17],
            ),
            status = NativeInsStatus.entries[statusIndex],
        )
    }

    internal fun loadAbsoluteForTests(path: String) {
        System.load(path)
        loaded = true
    }

    private fun InsState.pack(): DoubleArray = DoubleArray(STATE_SIZE).also { packed ->
        positionEnuMeters.copyInto(packed, 0)
        velocityEnuMetersPerSecond.copyInto(packed, 3)
        packed[6] = attitudeBodyToEnu.w
        packed[7] = attitudeBodyToEnu.x
        packed[8] = attitudeBodyToEnu.y
        packed[9] = attitudeBodyToEnu.z
        accelerometerBiasBodyMetersPerSecondSquared.copyInto(packed, 10)
        gyroBiasBodyRadiansPerSecond.copyInto(packed, 13)
        packed[16] = clockBiasMeters
        packed[17] = clockDriftMetersPerSecond
    }

    private fun ImuSample.pack(): DoubleArray = DoubleArray(SAMPLE_SIZE).also { packed ->
        specificForceBodyMetersPerSecondSquared.copyInto(packed, 0)
        angularRateBodyRadiansPerSecond.copyInto(packed, 3)
    }

    private external fun nativePropagate(
        stateTimestampNanos: Long,
        packedState: DoubleArray,
        previousTimestampNanos: Long,
        packedPrevious: DoubleArray,
        currentTimestampNanos: Long,
        packedCurrent: DoubleArray,
    ): DoubleArray
}
