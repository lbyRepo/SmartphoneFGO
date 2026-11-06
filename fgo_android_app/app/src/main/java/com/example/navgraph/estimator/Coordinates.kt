package com.example.navgraph.estimator

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

object Coordinates {
    private const val WGS84_A_METERS = 6_378_137.0
    private const val WGS84_FLATTENING = 1.0 / 298.257223563
    private val eccentricitySquared = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING)

    fun llaToEcef(latitudeDegrees: Double, longitudeDegrees: Double, altitudeMeters: Double): DoubleArray {
        val latitude = Math.toRadians(latitudeDegrees)
        val longitude = Math.toRadians(longitudeDegrees)
        val normal = WGS84_A_METERS / sqrt(1.0 - eccentricitySquared * sin(latitude) * sin(latitude))
        return doubleArrayOf(
            (normal + altitudeMeters) * cos(latitude) * cos(longitude),
            (normal + altitudeMeters) * cos(latitude) * sin(longitude),
            (normal * (1.0 - eccentricitySquared) + altitudeMeters) * sin(latitude),
        )
    }

    fun ecefToLla(positionMeters: DoubleArray): DoubleArray {
        val x = positionMeters[0]
        val y = positionMeters[1]
        val z = positionMeters[2]
        val longitude = atan2(y, x)
        val horizontal = hypot(x, y)
        var latitude = atan2(z, horizontal * (1.0 - eccentricitySquared))
        var altitude = 0.0
        repeat(12) {
            val normal = WGS84_A_METERS / sqrt(1.0 - eccentricitySquared * sin(latitude) * sin(latitude))
            altitude = horizontal / cos(latitude).coerceAtLeast(1.0e-12) - normal
            val updated = atan2(
                z,
                horizontal * (1.0 - eccentricitySquared * normal / (normal + altitude)),
            )
            if (kotlin.math.abs(updated - latitude) < 1.0e-13) {
                latitude = updated
                return@repeat
            }
            latitude = updated
        }
        return doubleArrayOf(Math.toDegrees(latitude), Math.toDegrees(longitude), altitude)
    }

    fun ecefDeltaToEnu(deltaMeters: DoubleArray, referenceEcefMeters: DoubleArray): DoubleArray {
        val lla = ecefToLla(referenceEcefMeters)
        val latitude = Math.toRadians(lla[0])
        val longitude = Math.toRadians(lla[1])
        val east = -sin(longitude) * deltaMeters[0] + cos(longitude) * deltaMeters[1]
        val north = -sin(latitude) * cos(longitude) * deltaMeters[0] -
            sin(latitude) * sin(longitude) * deltaMeters[1] + cos(latitude) * deltaMeters[2]
        val up = cos(latitude) * cos(longitude) * deltaMeters[0] +
            cos(latitude) * sin(longitude) * deltaMeters[1] + sin(latitude) * deltaMeters[2]
        return doubleArrayOf(east, north, up)
    }
}
