package com.example.navgraph.estimator

import kotlin.math.max
import kotlin.math.sqrt

data class EkfHealth(
    var acceptedPseudorange: Int = 0,
    var rejectedPseudorange: Int = 0,
    var acceptedDoppler: Int = 0,
    var rejectedDoppler: Int = 0,
    var propagationSteps: Int = 0,
    var nonMonotonicImuSamples: Int = 0,
    var nativePropagationSteps: Int = 0,
    var nativeGnssUpdates: Int = 0,
    var nativeFailures: Int = 0,
) {
    val rejectedMeasurements: Int
        get() = rejectedPseudorange + rejectedDoppler
}

data class EkfTranslationalVariance(
    val positionEnuMetersSquared: DoubleArray,
    val velocityEnuMetersPerSecondSquared: DoubleArray,
    val clockBiasMetersSquared: Double,
    val clockDriftMetersPerSecondSquared: Double,
)

class GnssInsEkf(
    private val frame: EnuFrame,
    initialState: InsState,
    initialCovariance: Array<DoubleArray> = defaultInitialCovariance(),
    private val applyEarthRotation: Boolean = true,
    private val nominalPropagator: (InsState, ImuSample, ImuSample) -> InsState =
        ImuMechanization::propagate,
    private val preferNative: Boolean = false,
) {
    var state: InsState = initialState.deepCopy()
        private set
    var covariance: Array<DoubleArray> = Array(ERROR_STATE_SIZE) { initialCovariance[it].copyOf() }
        private set
    val health = EkfHealth()
    var engine: String = if (preferNative) NativeGnssInsEkf.readinessLabel else "KOTLIN"
        private set

    init {
        require(initialCovariance.size == ERROR_STATE_SIZE &&
            initialCovariance.all { it.size == ERROR_STATE_SIZE })
        symmetrize(covariance)
    }

    fun propagate(previous: ImuSample, current: ImuSample) {
        if (current.timestampNanos <= previous.timestampNanos || state.timestampNanos != previous.timestampNanos) {
            health.nonMonotonicImuSamples++
            return
        }
        if (preferNative && NativeGnssInsEkf.isAvailable) {
            val native = runCatching {
                NativeGnssInsEkf.propagate(state, covariance, previous, current)
            }.getOrNull()
            if (native?.status == NativeEkfStatus.OK &&
                native.state.timestampNanos == current.timestampNanos &&
                native.state.numericallyValid() && native.covariance.numericallyValid()
            ) {
                state = native.state
                covariance = native.covariance
                health.propagationSteps += native.health.propagationSteps
                health.nativePropagationSteps += native.health.propagationSteps
                engine = "C++ JNI"
                return
            }
            health.nativeFailures++
            engine = "KOTLIN FALLBACK"
        }
        val dt = (current.timestampNanos - previous.timestampNanos) * 1.0e-9
        val angularRate = DoubleArray(3) {
            0.5 * (previous.angularRateBodyRadiansPerSecond[it] +
                current.angularRateBodyRadiansPerSecond[it]) - state.gyroBiasBodyRadiansPerSecond[it]
        }
        val midpointAttitude = state.attitudeBodyToEnu * UnitQuaternion.fromRotationVector(
            DoubleArray(3) { 0.5 * angularRate[it] * dt },
        )
        val specificForce = DoubleArray(3) {
            0.5 * (previous.specificForceBodyMetersPerSecondSquared[it] +
                current.specificForceBodyMetersPerSecondSquared[it]) -
                state.accelerometerBiasBodyMetersPerSecondSquared[it]
        }
        propagateCovariance(dt, midpointAttitude.rotationMatrix(), specificForce)
        state = nominalPropagator(state, previous, current)
        health.propagationSteps++
    }

    fun update(epoch: GnssEpoch) {
        require(kotlin.math.abs(epoch.integrationTimestampNanos - state.timestampNanos) <= 1_000_000L) {
            "GNSS epoch and INS state are not time-aligned"
        }
        if (preferNative && NativeGnssInsEkf.isAvailable) {
            val native = runCatching {
                NativeGnssInsEkf.update(frame, state, covariance, epoch, applyEarthRotation)
            }.getOrNull()
            if (native?.status == NativeEkfStatus.OK && native.state.numericallyValid() &&
                native.covariance.numericallyValid()
            ) {
                state = native.state
                covariance = native.covariance
                health.acceptedPseudorange += native.health.acceptedPseudorange
                health.rejectedPseudorange += native.health.rejectedPseudorange
                health.acceptedDoppler += native.health.acceptedDoppler
                health.rejectedDoppler += native.health.rejectedDoppler
                health.nativeGnssUpdates++
                engine = "C++ JNI"
                return
            }
            health.nativeFailures++
            engine = "KOTLIN FALLBACK"
        }
        epoch.observations.forEach { observation ->
            val receiverPosition = frame.enuPositionToEcef(state.positionEnuMeters)
            val receiverVelocity = frame.enuVectorToEcef(state.velocityEnuMetersPerSecond)
            val prediction = runCatching {
                GnssGeometry.predict(
                    receiverPosition,
                    receiverVelocity,
                    state.clockBiasMeters,
                    state.clockDriftMetersPerSecond,
                    observation,
                    applyEarthRotation,
                )
            }.getOrNull()
            if (prediction == null) {
                health.rejectedPseudorange++
                if (observation.correctedRangeRateMetersPerSecond.isFinite() &&
                    observation.rangeRateSigmaMetersPerSecond.isFinite()
                ) health.rejectedDoppler++
                return@forEach
            }
            val pseudorangeRow = DoubleArray(ERROR_STATE_SIZE)
            frame.rangeDerivativeEcefToEnu(prediction.rangeDerivativeEcef)
                .copyInto(pseudorangeRow, POSITION_INDEX)
            pseudorangeRow[CLOCK_BIAS_INDEX] = 1.0
            val pseudorangeAccepted = scalarUpdate(
                observation.correctedPseudorangeMeters - prediction.pseudorangeMeters,
                pseudorangeRow,
                max(observation.pseudorangeSigmaMeters, 1.0),
                PSEUDORANGE_GATE_SQUARED,
            )
            if (pseudorangeAccepted) health.acceptedPseudorange++ else health.rejectedPseudorange++

            if (observation.correctedRangeRateMetersPerSecond.isFinite() &&
                observation.rangeRateSigmaMetersPerSecond.isFinite()
            ) {
                val updatedPosition = frame.enuPositionToEcef(state.positionEnuMeters)
                val updatedVelocity = frame.enuVectorToEcef(state.velocityEnuMetersPerSecond)
                val updatedPrediction = GnssGeometry.predict(
                    updatedPosition,
                    updatedVelocity,
                    state.clockBiasMeters,
                    state.clockDriftMetersPerSecond,
                    observation,
                    applyEarthRotation,
                )
                val dopplerRow = DoubleArray(ERROR_STATE_SIZE)
                frame.rangeDerivativeEcefToEnu(updatedPrediction.rangeRatePositionDerivativeEcef)
                    .copyInto(dopplerRow, POSITION_INDEX)
                frame.rangeDerivativeEcefToEnu(updatedPrediction.rangeDerivativeEcef)
                    .copyInto(dopplerRow, VELOCITY_INDEX)
                dopplerRow[CLOCK_DRIFT_INDEX] = 1.0
                val dopplerAccepted = scalarUpdate(
                    observation.correctedRangeRateMetersPerSecond -
                        updatedPrediction.rangeRateMetersPerSecond,
                    dopplerRow,
                    max(observation.rangeRateSigmaMetersPerSecond, 0.15),
                    DOPPLER_GATE_SQUARED,
                )
                if (dopplerAccepted) health.acceptedDoppler++ else health.rejectedDoppler++
            }
        }
    }

    fun positionStandardDeviationMeters(): DoubleArray = DoubleArray(3) {
        sqrt(max(covariance[POSITION_INDEX + it][POSITION_INDEX + it], 0.0))
    }

    fun translationalVariance(): EkfTranslationalVariance = EkfTranslationalVariance(
        positionEnuMetersSquared = DoubleArray(3) {
            covariance[POSITION_INDEX + it][POSITION_INDEX + it]
        },
        velocityEnuMetersPerSecondSquared = DoubleArray(3) {
            covariance[VELOCITY_INDEX + it][VELOCITY_INDEX + it]
        },
        clockBiasMetersSquared = covariance[CLOCK_BIAS_INDEX][CLOCK_BIAS_INDEX],
        clockDriftMetersPerSecondSquared = covariance[CLOCK_DRIFT_INDEX][CLOCK_DRIFT_INDEX],
    )

    /**
     * Applies a bounded delayed-smoother delta without reducing covariance. EKF
     * and FGO consume the same measurements, so treating this as an independent
     * Kalman observation would create unjustified confidence.
     */
    fun applyCorrelatedSmootherDelta(
        positionDeltaEnuMeters: DoubleArray,
        velocityDeltaEnuMetersPerSecond: DoubleArray,
        clockBiasDeltaMeters: Double,
        clockDriftDeltaMetersPerSecond: Double,
        gain: Double,
    ) {
        require(positionDeltaEnuMeters.size == 3 && velocityDeltaEnuMetersPerSecond.size == 3)
        require(gain in 0.0..1.0)
        require(positionDeltaEnuMeters.all(Double::isFinite))
        require(velocityDeltaEnuMetersPerSecond.all(Double::isFinite))
        require(clockBiasDeltaMeters.isFinite() && clockDriftDeltaMetersPerSecond.isFinite())
        state = state.copy(
            positionEnuMeters = DoubleArray(3) {
                state.positionEnuMeters[it] + gain * positionDeltaEnuMeters[it]
            },
            velocityEnuMetersPerSecond = DoubleArray(3) {
                state.velocityEnuMetersPerSecond[it] + gain * velocityDeltaEnuMetersPerSecond[it]
            },
            clockBiasMeters = state.clockBiasMeters + gain * clockBiasDeltaMeters,
            clockDriftMetersPerSecond = state.clockDriftMetersPerSecond +
                gain * clockDriftDeltaMetersPerSecond,
        )
    }

    private fun propagateCovariance(
        dt: Double,
        bodyToEnuRotation: Array<DoubleArray>,
        specificForceBody: DoubleArray,
    ) {
        data class TransitionEntry(val row: Int, val column: Int, val value: Double)
        val transitionEntries = mutableListOf<TransitionEntry>()
        for (axis in 0..2) {
            transitionEntries += TransitionEntry(POSITION_INDEX + axis, VELOCITY_INDEX + axis, dt)
        }
        val accelerationAttitudeJacobian = multiplyMatrices(
            bodyToEnuRotation,
            skew(specificForceBody),
        )
        for (row in 0..2) {
            for (column in 0..2) {
                val attitudeDerivative = -accelerationAttitudeJacobian[row][column]
                transitionEntries += TransitionEntry(
                    POSITION_INDEX + row,
                    ATTITUDE_INDEX + column,
                    0.5 * attitudeDerivative * dt * dt,
                )
                transitionEntries += TransitionEntry(
                    VELOCITY_INDEX + row,
                    ATTITUDE_INDEX + column,
                    attitudeDerivative * dt,
                )
                transitionEntries += TransitionEntry(
                    POSITION_INDEX + row,
                    ACCEL_BIAS_INDEX + column,
                    -0.5 * bodyToEnuRotation[row][column] * dt * dt,
                )
                transitionEntries += TransitionEntry(
                    VELOCITY_INDEX + row,
                    ACCEL_BIAS_INDEX + column,
                    -bodyToEnuRotation[row][column] * dt,
                )
            }
            transitionEntries += TransitionEntry(ATTITUDE_INDEX + row, GYRO_BIAS_INDEX + row, -dt)
        }
        transitionEntries += TransitionEntry(CLOCK_BIAS_INDEX, CLOCK_DRIFT_INDEX, dt)

        // F is identity plus a small set of process couplings. Applying only those
        // entries avoids two dense 17x17 multiplies for every 100 Hz IMU sample.
        val leftProduct = Array(ERROR_STATE_SIZE) { covariance[it].copyOf() }
        transitionEntries.forEach { entry ->
            for (column in 0 until ERROR_STATE_SIZE) {
                leftProduct[entry.row][column] += entry.value * covariance[entry.column][column]
            }
        }
        val propagated = Array(ERROR_STATE_SIZE) { leftProduct[it].copyOf() }
        transitionEntries.forEach { entry ->
            for (row in 0 until ERROR_STATE_SIZE) {
                propagated[row][entry.row] += leftProduct[row][entry.column] * entry.value
            }
        }
        covariance = propagated
        val accelerationNoise = 0.08
        val gyroNoise = 0.004
        val accelerometerBiasWalk = 0.002
        val gyroBiasWalk = 0.0002
        for (axis in 0..2) {
            covariance[POSITION_INDEX + axis][POSITION_INDEX + axis] +=
                0.25 * accelerationNoise * accelerationNoise * dt * dt * dt * dt
            covariance[VELOCITY_INDEX + axis][VELOCITY_INDEX + axis] +=
                accelerationNoise * accelerationNoise * dt * dt
            covariance[ATTITUDE_INDEX + axis][ATTITUDE_INDEX + axis] +=
                gyroNoise * gyroNoise * dt
            covariance[ACCEL_BIAS_INDEX + axis][ACCEL_BIAS_INDEX + axis] +=
                accelerometerBiasWalk * accelerometerBiasWalk * dt
            covariance[GYRO_BIAS_INDEX + axis][GYRO_BIAS_INDEX + axis] +=
                gyroBiasWalk * gyroBiasWalk * dt
        }
        covariance[CLOCK_BIAS_INDEX][CLOCK_BIAS_INDEX] += 4.0 * dt
        covariance[CLOCK_DRIFT_INDEX][CLOCK_DRIFT_INDEX] += 0.04 * dt
        symmetrize(covariance)
    }

    private fun scalarUpdate(
        innovation: Double,
        measurementRow: DoubleArray,
        sigma: Double,
        gateSquared: Double,
    ): Boolean {
        if (!innovation.isFinite() || !sigma.isFinite() || sigma <= 0.0 ||
            measurementRow.any { !it.isFinite() }
        ) return false
        val projectedCovariance = DoubleArray(ERROR_STATE_SIZE) { row ->
            measurementRow.indices.sumOf { column -> covariance[row][column] * measurementRow[column] }
        }
        val innovationVariance = measurementRow.indices.sumOf {
            measurementRow[it] * projectedCovariance[it]
        } + sigma * sigma
        if (!innovationVariance.isFinite() || innovationVariance <= 1.0e-12 ||
            innovation * innovation / innovationVariance > gateSquared
        ) return false

        val gain = DoubleArray(ERROR_STATE_SIZE) { projectedCovariance[it] / innovationVariance }
        val correction = DoubleArray(ERROR_STATE_SIZE) { gain[it] * innovation }
        applyCorrection(correction)

        val josephLeft = identityMatrix(ERROR_STATE_SIZE)
        for (row in 0 until ERROR_STATE_SIZE) {
            for (column in 0 until ERROR_STATE_SIZE) {
                josephLeft[row][column] -= gain[row] * measurementRow[column]
            }
        }
        val updated = multiplyMatrices(
            multiplyMatrices(josephLeft, covariance),
            transposeMatrix(josephLeft),
        )
        val variance = sigma * sigma
        for (row in 0 until ERROR_STATE_SIZE) {
            for (column in 0 until ERROR_STATE_SIZE) {
                updated[row][column] += gain[row] * variance * gain[column]
            }
        }
        covariance = updated
        symmetrize(covariance)
        return true
    }

    private fun applyCorrection(error: DoubleArray) {
        state = state.copy(
            positionEnuMeters = DoubleArray(3) {
                state.positionEnuMeters[it] + error[POSITION_INDEX + it]
            },
            velocityEnuMetersPerSecond = DoubleArray(3) {
                state.velocityEnuMetersPerSecond[it] + error[VELOCITY_INDEX + it]
            },
            attitudeBodyToEnu = UnitQuaternion.fromRotationVector(
                error.copyOfRange(ATTITUDE_INDEX, ATTITUDE_INDEX + 3),
            ) * state.attitudeBodyToEnu,
            accelerometerBiasBodyMetersPerSecondSquared = DoubleArray(3) {
                state.accelerometerBiasBodyMetersPerSecondSquared[it] + error[ACCEL_BIAS_INDEX + it]
            },
            gyroBiasBodyRadiansPerSecond = DoubleArray(3) {
                state.gyroBiasBodyRadiansPerSecond[it] + error[GYRO_BIAS_INDEX + it]
            },
            clockBiasMeters = state.clockBiasMeters + error[CLOCK_BIAS_INDEX],
            clockDriftMetersPerSecond = state.clockDriftMetersPerSecond + error[CLOCK_DRIFT_INDEX],
        )
    }

    private fun InsState.deepCopy(): InsState = copy(
        positionEnuMeters = positionEnuMeters.copyOf(),
        velocityEnuMetersPerSecond = velocityEnuMetersPerSecond.copyOf(),
        accelerometerBiasBodyMetersPerSecondSquared =
            accelerometerBiasBodyMetersPerSecondSquared.copyOf(),
        gyroBiasBodyRadiansPerSecond = gyroBiasBodyRadiansPerSecond.copyOf(),
    )

    private fun InsState.numericallyValid(): Boolean =
        positionEnuMeters.all(Double::isFinite) &&
            velocityEnuMetersPerSecond.all(Double::isFinite) &&
            listOf(
                attitudeBodyToEnu.w,
                attitudeBodyToEnu.x,
                attitudeBodyToEnu.y,
                attitudeBodyToEnu.z,
            ).all(Double::isFinite) &&
            accelerometerBiasBodyMetersPerSecondSquared.all(Double::isFinite) &&
            gyroBiasBodyRadiansPerSecond.all(Double::isFinite) &&
            clockBiasMeters.isFinite() && clockDriftMetersPerSecond.isFinite()

    private fun Array<DoubleArray>.numericallyValid(): Boolean {
        if (size != ERROR_STATE_SIZE || any { it.size != ERROR_STATE_SIZE }) return false
        for (row in indices) {
            if (!this[row][row].isFinite() || this[row][row] < -1.0e-8) return false
            for (column in indices) {
                if (!this[row][column].isFinite() ||
                    kotlin.math.abs(this[row][column] - this[column][row]) > 1.0e-8
                ) return false
            }
        }
        return true
    }

    companion object {
        const val ERROR_STATE_SIZE = 17
        private const val POSITION_INDEX = 0
        private const val VELOCITY_INDEX = 3
        private const val ATTITUDE_INDEX = 6
        private const val ACCEL_BIAS_INDEX = 9
        private const val GYRO_BIAS_INDEX = 12
        private const val CLOCK_BIAS_INDEX = 15
        private const val CLOCK_DRIFT_INDEX = 16
        private const val PSEUDORANGE_GATE_SQUARED = 25.0
        private const val DOPPLER_GATE_SQUARED = 25.0

        fun defaultInitialCovariance(): Array<DoubleArray> =
            Array(ERROR_STATE_SIZE) { DoubleArray(ERROR_STATE_SIZE) }.also { covariance ->
                val standardDeviations = doubleArrayOf(
                    8.0, 8.0, 12.0,
                    2.0, 2.0, 2.0,
                    Math.toRadians(10.0), Math.toRadians(10.0), Math.toRadians(20.0),
                    0.15, 0.15, 0.15,
                    0.02, 0.02, 0.02,
                    100.0, 10.0,
                )
                standardDeviations.indices.forEach { index ->
                    covariance[index][index] = standardDeviations[index] * standardDeviations[index]
                }
            }
    }
}
