package com.example.navgraph.estimator

import java.util.Random
import kotlin.math.cos
import kotlin.math.sin

data class SyntheticTruthState(
    val timestampNanos: Long,
    val positionEnuMeters: DoubleArray,
    val velocityEnuMetersPerSecond: DoubleArray,
)

data class SyntheticGnssInsDataset(
    val frame: EnuFrame,
    val imuSamples: List<ImuSample>,
    val gnssEpochs: List<GnssEpoch>,
    val truthByTimestampMillis: LinkedHashMap<Long, DoubleArray>,
    val truthStates: List<SyntheticTruthState>,
)

object SyntheticGnssInsDatasetFactory {
    private const val IMU_RATE_HZ = 100
    private const val DURATION_SECONDS = 20
    private const val START_TIMESTAMP_NANOS = 10_000_000_000L

    fun create(seed: Long = 23L): SyntheticGnssInsDataset {
        val random = Random(seed)
        val reference = Coordinates.llaToEcef(1.3521, 103.8198, 35.0)
        val frame = EnuFrame(reference)
        val satellitePositions = satellitePositions(frame)
        val accelerometerBias = doubleArrayOf(0.020, -0.015, 0.030)
        val gyroBias = doubleArrayOf(0.0003, -0.0002, 0.0004)
        val imu = mutableListOf<ImuSample>()
        val epochs = mutableListOf<GnssEpoch>()
        val truthEcef = linkedMapOf<Long, DoubleArray>()
        val truthStates = mutableListOf<SyntheticTruthState>()
        var position = DoubleArray(3)
        var velocity = doubleArrayOf(0.8, 0.2, 0.0)
        var previousAcceleration = accelerationAt(0.0)
        val stepSeconds = 1.0 / IMU_RATE_HZ

        for (index in 0..DURATION_SECONDS * IMU_RATE_HZ) {
            val elapsedSeconds = index * stepSeconds
            val timestampNanos = START_TIMESTAMP_NANOS + (elapsedSeconds * 1.0e9).toLong()
            val acceleration = accelerationAt(elapsedSeconds)
            if (index > 0) {
                val midpointAcceleration = DoubleArray(3) {
                    0.5 * (previousAcceleration[it] + acceleration[it])
                }
                position = DoubleArray(3) {
                    position[it] + velocity[it] * stepSeconds +
                        0.5 * midpointAcceleration[it] * stepSeconds * stepSeconds
                }
                velocity = DoubleArray(3) {
                    velocity[it] + midpointAcceleration[it] * stepSeconds
                }
            }
            previousAcceleration = acceleration
            val specificForce = doubleArrayOf(
                acceleration[0] + accelerometerBias[0] + random.nextGaussian() * 0.015,
                acceleration[1] + accelerometerBias[1] + random.nextGaussian() * 0.015,
                acceleration[2] + ImuMechanization.STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED +
                    accelerometerBias[2] + random.nextGaussian() * 0.015,
            )
            val angularRate = DoubleArray(3) {
                gyroBias[it] + random.nextGaussian() * 0.00025
            }
            imu += ImuSample(timestampNanos, specificForce, angularRate)

            if (index % IMU_RATE_HZ == 0) {
                val timestampMillis = timestampNanos / 1_000_000L
                val receiverEcef = frame.enuPositionToEcef(position)
                val receiverVelocityEcef = frame.enuVectorToEcef(velocity)
                truthEcef[timestampMillis] = receiverEcef
                truthStates += SyntheticTruthState(
                    timestampNanos,
                    position.copyOf(),
                    velocity.copyOf(),
                )
                epochs += GnssEpoch(
                    timestampMillis,
                    satellitePositions.mapIndexed { satelliteIndex, satellitePosition ->
                        observation(
                            timestampMillis,
                            elapsedSeconds,
                            satelliteIndex,
                            satellitePosition,
                            receiverEcef,
                            receiverVelocityEcef,
                            random,
                        )
                    },
                )
            }
        }
        return SyntheticGnssInsDataset(frame, imu, epochs, truthEcef, truthStates)
    }

    private fun accelerationAt(seconds: Double): DoubleArray = when {
        seconds < 4.0 -> doubleArrayOf(0.35, 0.0, 0.0)
        seconds < 8.0 -> doubleArrayOf(0.0, 0.0, 0.0)
        seconds < 12.0 -> doubleArrayOf(-0.20, 0.22, 0.0)
        seconds < 16.0 -> doubleArrayOf(0.0, -0.12, 0.0)
        else -> doubleArrayOf(0.0, 0.0, 0.0)
    }

    private fun satellitePositions(frame: EnuFrame): List<DoubleArray> {
        val azimuthElevationDegrees = listOf(
            15.0 to 25.0,
            70.0 to 45.0,
            125.0 to 30.0,
            180.0 to 65.0,
            225.0 to 35.0,
            275.0 to 55.0,
            320.0 to 28.0,
            350.0 to 72.0,
        )
        return azimuthElevationDegrees.map { (azimuthDegrees, elevationDegrees) ->
            val azimuth = Math.toRadians(azimuthDegrees)
            val elevation = Math.toRadians(elevationDegrees)
            val directionEnu = doubleArrayOf(
                cos(elevation) * sin(azimuth),
                cos(elevation) * cos(azimuth),
                sin(elevation),
            )
            val offsetEcef = frame.enuVectorToEcef(DoubleArray(3) { directionEnu[it] * 21_000_000.0 })
            DoubleArray(3) { frame.referenceEcefMeters[it] + offsetEcef[it] }
        }
    }

    private fun observation(
        timestampMillis: Long,
        elapsedSeconds: Double,
        satelliteIndex: Int,
        satellitePosition: DoubleArray,
        receiverPosition: DoubleArray,
        receiverVelocity: DoubleArray,
        random: Random,
    ): GnssObservation {
        val delta = subtract(receiverPosition, satellitePosition)
        val distance = norm(delta)
        val lineOfSight = DoubleArray(3) { delta[it] / distance }
        val clockBias = 85.0 + 0.18 * elapsedSeconds
        val clockDrift = 0.18
        val pseudorangeOutlier = if (elapsedSeconds == 12.0 && satelliteIndex == 2) 38.0 else 0.0
        val pseudorange = distance + clockBias + random.nextGaussian() * 2.2 + pseudorangeOutlier
        val rangeRate = lineOfSight.indices.sumOf { lineOfSight[it] * receiverVelocity[it] } +
            clockDrift + random.nextGaussian() * 0.08
        return GnssObservation(
            timestampMillis = timestampMillis,
            constellation = 1,
            svid = satelliteIndex + 1,
            signalType = "SYNTHETIC_L1",
            satellitePositionMeters = satellitePosition,
            satelliteVelocityMetersPerSecond = DoubleArray(3),
            correctedPseudorangeMeters = pseudorange,
            pseudorangeSigmaMeters = 2.5,
            correctedRangeRateMetersPerSecond = rangeRate,
            rangeRateSigmaMetersPerSecond = 0.15,
            cn0DbHz = 38.0 + satelliteIndex,
            elevationDegrees = 25.0 + satelliteIndex * 5.0,
        )
    }
}
