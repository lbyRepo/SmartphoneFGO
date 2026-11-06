package com.example.navgraph.estimator

import kotlin.math.max

data class ImuSample(
    val timestampNanos: Long,
    val specificForceBodyMetersPerSecondSquared: DoubleArray,
    val angularRateBodyRadiansPerSecond: DoubleArray,
)

data class InsState(
    val timestampNanos: Long,
    val positionEnuMeters: DoubleArray,
    val velocityEnuMetersPerSecond: DoubleArray,
    val attitudeBodyToEnu: UnitQuaternion,
    val accelerometerBiasBodyMetersPerSecondSquared: DoubleArray,
    val gyroBiasBodyRadiansPerSecond: DoubleArray,
    val clockBiasMeters: Double,
    val clockDriftMetersPerSecond: Double,
)

data class ImuPreintegration(
    val deltaTimeSeconds: Double,
    val deltaPositionEnuMeters: DoubleArray,
    val deltaVelocityEnuMetersPerSecond: DoubleArray,
    val deltaAttitude: UnitQuaternion,
    val sampleCount: Int,
)

object ImuMechanization {
    const val STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665
    val gravityEnuMetersPerSecondSquared = doubleArrayOf(
        0.0,
        0.0,
        -STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED,
    )

    fun propagate(state: InsState, previous: ImuSample, current: ImuSample): InsState {
        require(current.timestampNanos > previous.timestampNanos) { "IMU timestamps must increase" }
        require(state.timestampNanos == previous.timestampNanos) {
            "INS state and previous IMU sample must share a timestamp"
        }
        val dt = (current.timestampNanos - previous.timestampNanos) * 1.0e-9
        val angularRate = DoubleArray(3) {
            0.5 * (previous.angularRateBodyRadiansPerSecond[it] +
                current.angularRateBodyRadiansPerSecond[it]) - state.gyroBiasBodyRadiansPerSecond[it]
        }
        val halfStepAttitude = state.attitudeBodyToEnu * UnitQuaternion.fromRotationVector(
            DoubleArray(3) { 0.5 * angularRate[it] * dt },
        )
        val specificForce = DoubleArray(3) {
            0.5 * (previous.specificForceBodyMetersPerSecondSquared[it] +
                current.specificForceBodyMetersPerSecondSquared[it]) -
                state.accelerometerBiasBodyMetersPerSecondSquared[it]
        }
        val accelerationEnu = halfStepAttitude.rotate(specificForce).also { acceleration ->
            for (axis in 0..2) acceleration[axis] += gravityEnuMetersPerSecondSquared[axis]
        }
        val position = DoubleArray(3) {
            state.positionEnuMeters[it] + state.velocityEnuMetersPerSecond[it] * dt +
                0.5 * accelerationEnu[it] * dt * dt
        }
        val velocity = DoubleArray(3) {
            state.velocityEnuMetersPerSecond[it] + accelerationEnu[it] * dt
        }
        val attitude = state.attitudeBodyToEnu * UnitQuaternion.fromRotationVector(
            DoubleArray(3) { angularRate[it] * dt },
        )
        return state.copy(
            timestampNanos = current.timestampNanos,
            positionEnuMeters = position,
            velocityEnuMetersPerSecond = velocity,
            attitudeBodyToEnu = attitude,
            clockBiasMeters = state.clockBiasMeters + state.clockDriftMetersPerSecond * dt,
        )
    }

    fun preintegrate(
        samples: List<ImuSample>,
        initialAttitudeBodyToEnu: UnitQuaternion,
        accelerometerBiasBody: DoubleArray = DoubleArray(3),
        gyroBiasBody: DoubleArray = DoubleArray(3),
    ): ImuPreintegration {
        require(samples.size >= 2) { "Preintegration needs at least two IMU samples" }
        var state = InsState(
            timestampNanos = samples.first().timestampNanos,
            positionEnuMeters = DoubleArray(3),
            velocityEnuMetersPerSecond = DoubleArray(3),
            attitudeBodyToEnu = initialAttitudeBodyToEnu,
            accelerometerBiasBodyMetersPerSecondSquared = accelerometerBiasBody.copyOf(),
            gyroBiasBodyRadiansPerSecond = gyroBiasBody.copyOf(),
            clockBiasMeters = 0.0,
            clockDriftMetersPerSecond = 0.0,
        )
        for (index in 1 until samples.size) {
            state = propagate(state, samples[index - 1], samples[index])
        }
        return ImuPreintegration(
            deltaTimeSeconds = max(
                (samples.last().timestampNanos - samples.first().timestampNanos) * 1.0e-9,
                0.0,
            ),
            deltaPositionEnuMeters = state.positionEnuMeters,
            deltaVelocityEnuMetersPerSecond = state.velocityEnuMetersPerSecond,
            deltaAttitude = initialAttitudeBodyToEnu.conjugate() * state.attitudeBodyToEnu,
            sampleCount = samples.size,
        )
    }
}
