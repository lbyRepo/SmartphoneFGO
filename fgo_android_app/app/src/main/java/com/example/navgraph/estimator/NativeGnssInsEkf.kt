package com.example.navgraph.estimator

enum class NativeEkfStatus {
    OK,
    NON_INCREASING_TIMESTAMP,
    STATE_TIMESTAMP_MISMATCH,
    NON_FINITE_INPUT,
    INVALID_QUATERNION,
    GNSS_TIMESTAMP_MISMATCH,
    INVALID_FRAME,
}

data class NativeEkfHealthDelta(
    val propagationSteps: Int,
    val acceptedPseudorange: Int,
    val rejectedPseudorange: Int,
    val acceptedDoppler: Int,
    val rejectedDoppler: Int,
)

data class NativeEkfStepResult(
    val state: InsState,
    val covariance: Array<DoubleArray>,
    val status: NativeEkfStatus,
    val health: NativeEkfHealthDelta,
)

/** Complete portable 17-state EKF step. Kotlin is the explicit fallback path. */
object NativeGnssInsEkf {
    private const val STATE_SIZE = 18
    private const val ERROR_STATE_SIZE = 17
    private const val COVARIANCE_SIZE = ERROR_STATE_SIZE * ERROR_STATE_SIZE
    private const val OBSERVATION_STRIDE = 10
    private const val STATUS_OFFSET = STATE_SIZE + COVARIANCE_SIZE
    private const val RESULT_SIZE = STATUS_OFFSET + 6

    @Volatile
    private var loaded = runCatching { System.loadLibrary("navgraph_estimator") }.isSuccess

    val isAvailable: Boolean
        get() = loaded

    val readinessLabel: String
        get() = if (loaded) "C++ JNI READY" else "KOTLIN"

    fun propagate(
        state: InsState,
        covariance: Array<DoubleArray>,
        previous: ImuSample,
        current: ImuSample,
    ): NativeEkfStepResult? {
        if (!loaded) return null
        return unpackResult(
            nativePropagate(
                state.timestampNanos,
                state.pack(),
                covariance.pack(),
                previous.timestampNanos,
                previous.pack(),
                current.timestampNanos,
                current.pack(),
            ),
            successTimestampNanos = current.timestampNanos,
            failureTimestampNanos = state.timestampNanos,
        )
    }

    fun update(
        frame: EnuFrame,
        state: InsState,
        covariance: Array<DoubleArray>,
        epoch: GnssEpoch,
        applyEarthRotation: Boolean = true,
    ): NativeEkfStepResult? {
        if (!loaded) return null
        val packedFrame = DoubleArray(12)
        frame.referenceEcefMeters.copyInto(packedFrame, 0)
        frame.ecefToEnuRowMajor().copyInto(packedFrame, 3)
        val observations = DoubleArray(epoch.observations.size * OBSERVATION_STRIDE)
        epoch.observations.forEachIndexed { index, observation ->
            val offset = index * OBSERVATION_STRIDE
            observation.satellitePositionMeters.copyInto(observations, offset)
            observation.satelliteVelocityMetersPerSecond.copyInto(observations, offset + 3)
            observations[offset + 6] = observation.correctedPseudorangeMeters
            observations[offset + 7] = observation.pseudorangeSigmaMeters
            observations[offset + 8] = observation.correctedRangeRateMetersPerSecond
            observations[offset + 9] = observation.rangeRateSigmaMetersPerSecond
        }
        return unpackResult(
            nativeUpdate(
                packedFrame,
                state.timestampNanos,
                state.pack(),
                covariance.pack(),
                epoch.timestampMillis,
                epoch.integrationTimestampNanos,
                observations,
                applyEarthRotation,
            ),
            successTimestampNanos = state.timestampNanos,
            failureTimestampNanos = state.timestampNanos,
        )
    }

    internal fun loadAbsoluteForTests(path: String) {
        System.load(path)
        loaded = true
    }

    private fun unpackResult(
        values: DoubleArray,
        successTimestampNanos: Long,
        failureTimestampNanos: Long,
    ): NativeEkfStepResult {
        require(values.size == RESULT_SIZE) { "Native EKF returned ${values.size} values" }
        val statusIndex = values[STATUS_OFFSET].toInt()
        require(statusIndex in NativeEkfStatus.entries.indices) {
            "Native EKF returned invalid status $statusIndex"
        }
        val covariance = Array(ERROR_STATE_SIZE) { row ->
            values.copyOfRange(
                STATE_SIZE + row * ERROR_STATE_SIZE,
                STATE_SIZE + (row + 1) * ERROR_STATE_SIZE,
            )
        }
        return NativeEkfStepResult(
            state = values.unpackState(
                if (statusIndex == NativeEkfStatus.OK.ordinal) {
                    successTimestampNanos
                } else {
                    failureTimestampNanos
                },
            ),
            covariance = covariance,
            status = NativeEkfStatus.entries[statusIndex],
            health = NativeEkfHealthDelta(
                propagationSteps = values[STATUS_OFFSET + 1].toInt(),
                acceptedPseudorange = values[STATUS_OFFSET + 2].toInt(),
                rejectedPseudorange = values[STATUS_OFFSET + 3].toInt(),
                acceptedDoppler = values[STATUS_OFFSET + 4].toInt(),
                rejectedDoppler = values[STATUS_OFFSET + 5].toInt(),
            ),
        )
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

    private fun DoubleArray.unpackState(timestampNanos: Long): InsState = InsState(
        timestampNanos = timestampNanos,
        positionEnuMeters = copyOfRange(0, 3),
        velocityEnuMetersPerSecond = copyOfRange(3, 6),
        attitudeBodyToEnu = UnitQuaternion(this[6], this[7], this[8], this[9]),
        accelerometerBiasBodyMetersPerSecondSquared = copyOfRange(10, 13),
        gyroBiasBodyRadiansPerSecond = copyOfRange(13, 16),
        clockBiasMeters = this[16],
        clockDriftMetersPerSecond = this[17],
    )

    private fun Array<DoubleArray>.pack(): DoubleArray {
        require(size == ERROR_STATE_SIZE && all { it.size == ERROR_STATE_SIZE })
        return DoubleArray(COVARIANCE_SIZE) { index ->
            this[index / ERROR_STATE_SIZE][index % ERROR_STATE_SIZE]
        }
    }

    private fun ImuSample.pack(): DoubleArray = DoubleArray(6).also { packed ->
        specificForceBodyMetersPerSecondSquared.copyInto(packed, 0)
        angularRateBodyRadiansPerSecond.copyInto(packed, 3)
    }

    private external fun nativePropagate(
        stateTimestampNanos: Long,
        packedState: DoubleArray,
        packedCovariance: DoubleArray,
        previousTimestampNanos: Long,
        packedPrevious: DoubleArray,
        currentTimestampNanos: Long,
        packedCurrent: DoubleArray,
    ): DoubleArray

    private external fun nativeUpdate(
        packedFrame: DoubleArray,
        stateTimestampNanos: Long,
        packedState: DoubleArray,
        packedCovariance: DoubleArray,
        timestampMillis: Long,
        integrationTimestampNanos: Long,
        packedObservations: DoubleArray,
        applyEarthRotation: Boolean,
    ): DoubleArray
}
