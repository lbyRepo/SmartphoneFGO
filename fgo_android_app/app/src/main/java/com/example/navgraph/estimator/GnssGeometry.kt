package com.example.navgraph.estimator

import kotlin.math.cos
import kotlin.math.sin

internal data class GnssPrediction(
    val pseudorangeMeters: Double,
    val rangeRateMetersPerSecond: Double,
    val rangeDerivativeEcef: DoubleArray,
    val rangeRatePositionDerivativeEcef: DoubleArray,
)

internal object GnssGeometry {
    private const val SPEED_OF_LIGHT_MPS = 299_792_458.0
    private const val EARTH_ROTATION_RADIANS_PER_SECOND = 7.2921151467e-5

    fun predict(
        receiverPositionEcefMeters: DoubleArray,
        receiverVelocityEcefMetersPerSecond: DoubleArray,
        clockBiasMeters: Double,
        clockDriftMetersPerSecond: Double,
        observation: GnssObservation,
        applyEarthRotation: Boolean,
    ): GnssPrediction {
        val transitSeconds = (observation.correctedPseudorangeMeters - clockBiasMeters) /
            SPEED_OF_LIGHT_MPS
        val satellitePosition = if (applyEarthRotation) {
            rotateSatellite(observation.satellitePositionMeters, transitSeconds)
        } else {
            observation.satellitePositionMeters
        }
        val delta = subtract(receiverPositionEcefMeters, satellitePosition)
        val distance = norm(delta)
        require(distance > 1.0)
        val lineOfSightDerivative = DoubleArray(3) { delta[it] / distance }
        val satelliteVelocity = observation.satelliteVelocityMetersPerSecond
        val relativeVelocity = DoubleArray(3) {
            receiverVelocityEcefMetersPerSecond[it] -
                if (satelliteVelocity[it].isFinite()) satelliteVelocity[it] else 0.0
        }
        val radialVelocity = dot(lineOfSightDerivative, relativeVelocity)
        val positionDerivative = DoubleArray(3) {
            (relativeVelocity[it] - lineOfSightDerivative[it] * radialVelocity) / distance
        }
        return GnssPrediction(
            pseudorangeMeters = distance + clockBiasMeters,
            rangeRateMetersPerSecond = radialVelocity + clockDriftMetersPerSecond,
            rangeDerivativeEcef = lineOfSightDerivative,
            rangeRatePositionDerivativeEcef = positionDerivative,
        )
    }

    private fun rotateSatellite(position: DoubleArray, transitSeconds: Double): DoubleArray {
        val angle = EARTH_ROTATION_RADIANS_PER_SECOND * transitSeconds
        val cosine = cos(angle)
        val sine = sin(angle)
        return doubleArrayOf(
            cosine * position[0] + sine * position[1],
            -sine * position[0] + cosine * position[1],
            position[2],
        )
    }

    private fun dot(left: DoubleArray, right: DoubleArray): Double =
        left.indices.sumOf { left[it] * right[it] }
}
