package com.example.navgraph.preprocessing

import com.example.navgraph.estimator.Coordinates
import com.example.navgraph.live.RawGnssObservationSample
import kotlin.math.asin
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

data class ObservationCorrections(
    val earthRotationMeters: Double,
    val ionosphereMeters: Double,
    val troposphereMeters: Double,
    val interSignalBiasMeters: Double,
    val uncertaintyMeters: Double,
    val atmosphericModelComplete: Boolean,
)

sealed interface CorrectionResult {
    data class Available(val corrections: ObservationCorrections) : CorrectionResult
    data class Unavailable(val reason: String) : CorrectionResult
}

fun interface CorrectionProvider {
    fun corrections(
        observation: RawGnssObservationSample,
        satellite: SatelliteState,
        approximateReceiverEcefMeters: DoubleArray?,
    ): CorrectionResult
}

/**
 * Standard-atmosphere troposphere plus an explicit conservative ionosphere budget.
 * Earth rotation remains in GnssGeometry, so it is intentionally not applied here.
 */
class ConservativeCorrectionProvider : CorrectionProvider {
    override fun corrections(
        observation: RawGnssObservationSample,
        satellite: SatelliteState,
        approximateReceiverEcefMeters: DoubleArray?,
    ): CorrectionResult {
        val interSignalBiasNanos = (observation.fullInterSignalBiasNanos ?: 0.0) +
            (observation.satelliteInterSignalBiasNanos ?: 0.0)
        val interSignalBiasMeters = interSignalBiasNanos * SPEED_OF_LIGHT_METERS_PER_NANOSECOND
        val receiver = approximateReceiverEcefMeters
        if (receiver == null || receiver.size != 3 || receiver.any { !it.isFinite() }) {
            return CorrectionResult.Available(ObservationCorrections(
                earthRotationMeters = 0.0,
                ionosphereMeters = 0.0,
                troposphereMeters = 0.0,
                interSignalBiasMeters = interSignalBiasMeters,
                uncertaintyMeters = hypot(15.0, satellite.accuracyMeters),
                atmosphericModelComplete = false,
            ))
        }
        val delta = DoubleArray(3) { satellite.positionEcefMeters[it] - receiver[it] }
        val enu = Coordinates.ecefDeltaToEnu(delta, receiver)
        val range = kotlin.math.sqrt(enu.sumOf { it * it })
        if (range < 1.0) return CorrectionResult.Unavailable("invalid_geometry")
        val elevation = asin((enu[2] / range).coerceIn(-1.0, 1.0))
        val mappedSine = max(sin(elevation), 0.10)
        val troposphere = 2.30 / mappedSine
        val ionosphereSigma = 7.5
        val troposphereSigma = 0.75 / mappedSine
        return CorrectionResult.Available(ObservationCorrections(
            earthRotationMeters = 0.0,
            ionosphereMeters = 0.0,
            troposphereMeters = troposphere,
            interSignalBiasMeters = interSignalBiasMeters,
            uncertaintyMeters = hypot(hypot(ionosphereSigma, troposphereSigma), satellite.accuracyMeters),
            atmosphericModelComplete = false,
        ))
    }

    private companion object {
        const val SPEED_OF_LIGHT_METERS_PER_NANOSECOND = 0.299792458
    }
}
