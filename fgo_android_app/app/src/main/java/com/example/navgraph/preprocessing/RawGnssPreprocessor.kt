package com.example.navgraph.preprocessing

import com.example.navgraph.estimator.Coordinates
import com.example.navgraph.estimator.GnssEpoch
import com.example.navgraph.estimator.GnssObservation
import com.example.navgraph.live.RawGnssEpochSample
import com.example.navgraph.live.RawGnssObservationSample
import kotlin.math.asin
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sqrt

data class PreprocessResult(
    val epoch: GnssEpoch?,
    val acceptedObservationCount: Int,
    val rejectionCounts: Map<String, Int>,
    val clockContinuityId: Int,
    val receiveGpsTimeNanos: Long?,
)

class RawGnssPreprocessor(
    private val satelliteStateProvider: SatelliteStateProvider,
    private val correctionProvider: CorrectionProvider,
) {
    private var approximateReceiverEcefMeters: DoubleArray? = null

    fun updateApproximateReceiver(latitudeDegrees: Double, longitudeDegrees: Double, altitudeMeters: Double) {
        approximateReceiverEcefMeters = Coordinates.llaToEcef(
            latitudeDegrees,
            longitudeDegrees,
            altitudeMeters,
        )
    }

    fun consume(input: RawGnssEpochSample): PreprocessResult {
        val rejected = linkedMapOf<String, Int>()
        val fullBiasNanos = input.clock.fullBiasNanos ?: return PreprocessResult(
            epoch = null,
            acceptedObservationCount = 0,
            rejectionCounts = mapOf(REJECTION_MISSING_CLOCK_BIAS to input.observations.size),
            clockContinuityId = input.clock.hardwareClockDiscontinuityCount,
            receiveGpsTimeNanos = null,
        )
        val biasNanos = input.clock.biasNanos ?: 0.0
        val receiveGpsTimeNanos = (input.clock.timeNanos - fullBiasNanos - biasNanos).toLong()
        val timestampMillis = receiveGpsTimeNanos / 1_000_000L
        val accepted = input.observations.mapNotNull { raw ->
            val prepared = prepareObservation(
                raw,
                receiveGpsTimeNanos,
                timestampMillis,
                input.clock.biasUncertaintyNanos,
            )
            if (prepared.observation == null) {
                val reason = prepared.rejectionReason ?: REJECTION_INVALID_PSEUDORANGE
                rejected[reason] = (rejected[reason] ?: 0) + 1
            }
            prepared.observation
        }
        if (accepted.size < 4) {
            rejected[REJECTION_INSUFFICIENT_OBSERVATIONS] = 1
        }
        return PreprocessResult(
            epoch = if (accepted.size >= 4) GnssEpoch(
                timestampMillis = timestampMillis,
                observations = accepted,
                elapsedRealtimeNanos = input.elapsedRealtimeNanos,
            ) else null,
            acceptedObservationCount = accepted.size,
            rejectionCounts = rejected,
            clockContinuityId = input.clock.hardwareClockDiscontinuityCount,
            receiveGpsTimeNanos = receiveGpsTimeNanos,
        )
    }

    private fun prepareObservation(
        raw: RawGnssObservationSample,
        receiveGpsTimeNanos: Long,
        timestampMillis: Long,
        clockBiasUncertaintyNanos: Double?,
    ): PreparedObservation {
        if (raw.constellation !in SUPPORTED_CONSTELLATIONS) {
            return PreparedObservation(rejectionReason = REJECTION_UNSUPPORTED_CONSTELLATION)
        }
        if (!hasUsableTrackingState(raw.state)) {
            return PreparedObservation(rejectionReason = REJECTION_INVALID_TRACKING_STATE)
        }
        val frequency = raw.carrierFrequencyHz
        if (frequency != null && (frequency !in MIN_FREQUENCY_HZ..MAX_FREQUENCY_HZ)) {
            return PreparedObservation(rejectionReason = REJECTION_UNSUPPORTED_FREQUENCY)
        }
        if (!raw.cn0DbHz.isFinite() || raw.cn0DbHz < MIN_CN0_DBHZ) {
            return PreparedObservation(rejectionReason = REJECTION_LOW_SIGNAL)
        }
        if (raw.receivedSvTimeUncertaintyNanos !in 0L..MAX_TRANSMIT_TIME_UNCERTAINTY_NANOS) {
            return PreparedObservation(rejectionReason = REJECTION_INVALID_TIME_UNCERTAINTY)
        }
        val time = pseudorangeTime(raw, receiveGpsTimeNanos)
            ?: return PreparedObservation(rejectionReason = REJECTION_INVALID_PSEUDORANGE)
        val signal = SatelliteSignalKey(raw.constellation, raw.svid, frequency, raw.codeType)
        val satellite = when (val result = satelliteStateProvider.stateAt(signal, time.transmitGpsTimeNanos)) {
            is SatelliteStateResult.Available -> result.state
            is SatelliteStateResult.Unavailable -> return PreparedObservation(
                rejectionReason = when (result.reason) {
                    SatelliteStateFailureReason.UNAVAILABLE_EPHEMERIS -> REJECTION_UNAVAILABLE_EPHEMERIS
                    SatelliteStateFailureReason.UNHEALTHY_SATELLITE -> REJECTION_UNHEALTHY_SATELLITE
                    SatelliteStateFailureReason.STALE_EPHEMERIS -> REJECTION_STALE_EPHEMERIS
                    SatelliteStateFailureReason.UNSUPPORTED_CONSTELLATION -> REJECTION_UNSUPPORTED_CONSTELLATION
                    SatelliteStateFailureReason.UNSUPPORTED_SIGNAL -> REJECTION_UNSUPPORTED_SIGNAL
                    SatelliteStateFailureReason.INVALID_TIME -> REJECTION_INVALID_PSEUDORANGE
                },
            )
        }
        val elevationDegrees = elevationDegrees(satellite.positionEcefMeters)
        if (elevationDegrees.isFinite() && elevationDegrees < MIN_ELEVATION_DEGREES) {
            return PreparedObservation(rejectionReason = REJECTION_LOW_ELEVATION)
        }
        val corrections = when (val result = correctionProvider.corrections(
            raw,
            satellite,
            approximateReceiverEcefMeters,
        )) {
            is CorrectionResult.Available -> result.corrections
            is CorrectionResult.Unavailable -> return PreparedObservation(REJECTION_MISSING_CORRECTION)
        }
        val correctedPseudorange = time.rawPseudorangeMeters + satellite.clockBiasMeters -
            corrections.interSignalBiasMeters - corrections.ionosphereMeters - corrections.troposphereMeters
        if (!correctedPseudorange.isFinite() || correctedPseudorange !in MIN_PSEUDORANGE_METERS..MAX_PSEUDORANGE_METERS) {
            return PreparedObservation(rejectionReason = REJECTION_INVALID_PSEUDORANGE)
        }
        val transmitSigmaMeters = raw.receivedSvTimeUncertaintyNanos * SPEED_OF_LIGHT_METERS_PER_NANOSECOND
        val clockSigmaMeters = (clockBiasUncertaintyNanos ?: 0.0) * SPEED_OF_LIGHT_METERS_PER_NANOSECOND
        val pseudorangeSigma = max(1.5, hypot(hypot(transmitSigmaMeters, clockSigmaMeters), corrections.uncertaintyMeters))
        val rangeRate = raw.pseudorangeRateMetersPerSecond + satellite.clockDriftMetersPerSecond
        val rangeRateSigma = if (raw.pseudorangeRateUncertaintyMetersPerSecond.isFinite() &&
            raw.pseudorangeRateUncertaintyMetersPerSecond > 0.0
        ) max(0.15, raw.pseudorangeRateUncertaintyMetersPerSecond) else Double.NaN
        return PreparedObservation(observation = GnssObservation(
            timestampMillis = timestampMillis,
            constellation = raw.constellation,
            svid = raw.svid,
            signalType = raw.codeType,
            satellitePositionMeters = satellite.positionEcefMeters.copyOf(),
            satelliteVelocityMetersPerSecond = satellite.velocityEcefMetersPerSecond.copyOf(),
            correctedPseudorangeMeters = correctedPseudorange,
            pseudorangeSigmaMeters = pseudorangeSigma,
            correctedRangeRateMetersPerSecond = if (rangeRateSigma.isFinite()) rangeRate else Double.NaN,
            rangeRateSigmaMetersPerSecond = rangeRateSigma,
            cn0DbHz = raw.cn0DbHz,
            elevationDegrees = elevationDegrees,
        ))
    }

    private fun pseudorangeTime(raw: RawGnssObservationSample, receiveGpsTimeNanos: Long): PseudorangeTime? {
        if (!raw.timeOffsetNanos.isFinite()) return null
        val observationReceiveGpsNanos = receiveGpsTimeNanos + raw.timeOffsetNanos
        val weekStart = Math.floor(observationReceiveGpsNanos / WEEK_NANOS).toLong() * WEEK_NANOS
        var receiveTowNanos = observationReceiveGpsNanos - weekStart
        var transmitTowGpsNanos = raw.receivedSvTimeNanos.toDouble()
        if (raw.constellation == CONSTELLATION_BEIDOU) {
            receiveTowNanos -= BEIDOU_TO_GPS_NANOS
            transmitTowGpsNanos += BEIDOU_TO_GPS_NANOS
        }
        var travelNanos = receiveTowNanos - raw.receivedSvTimeNanos
        if (travelNanos > HALF_WEEK_NANOS.toDouble()) travelNanos -= WEEK_NANOS.toDouble()
        if (travelNanos < -HALF_WEEK_NANOS.toDouble()) travelNanos += WEEK_NANOS.toDouble()
        val rawPseudorangeMeters = travelNanos * SPEED_OF_LIGHT_METERS_PER_NANOSECOND
        if (!rawPseudorangeMeters.isFinite() || rawPseudorangeMeters !in MIN_PSEUDORANGE_METERS..MAX_PSEUDORANGE_METERS) {
            return null
        }
        var transmitGpsTimeNanos = weekStart + transmitTowGpsNanos.toLong()
        while (transmitGpsTimeNanos > observationReceiveGpsNanos) transmitGpsTimeNanos -= WEEK_NANOS
        return PseudorangeTime(rawPseudorangeMeters, transmitGpsTimeNanos)
    }

    private fun elevationDegrees(satellitePosition: DoubleArray): Double {
        val receiver = approximateReceiverEcefMeters ?: return Double.NaN
        val delta = DoubleArray(3) { satellitePosition[it] - receiver[it] }
        val enu = Coordinates.ecefDeltaToEnu(delta, receiver)
        val range = sqrt(enu.sumOf { it * it })
        return if (range > 1.0) Math.toDegrees(asin((enu[2] / range).coerceIn(-1.0, 1.0))) else Double.NaN
    }

    private fun hasUsableTrackingState(state: Int): Boolean =
        state and STATE_CODE_LOCK != 0 && state and (STATE_TOW_DECODED or STATE_TOW_KNOWN) != 0

    private data class PseudorangeTime(
        val rawPseudorangeMeters: Double,
        val transmitGpsTimeNanos: Long,
    )

    private data class PreparedObservation(
        val rejectionReason: String? = null,
        val observation: GnssObservation? = null,
    )

    companion object {
        const val REJECTION_MISSING_CLOCK_BIAS = "missing_clock_bias"
        const val REJECTION_UNSUPPORTED_CONSTELLATION = "unsupported_constellation_or_time"
        const val REJECTION_INVALID_TRACKING_STATE = "invalid_tracking_state"
        const val REJECTION_UNSUPPORTED_FREQUENCY = "unsupported_frequency"
        const val REJECTION_UNSUPPORTED_SIGNAL = "unsupported_signal"
        const val REJECTION_LOW_SIGNAL = "low_signal"
        const val REJECTION_LOW_ELEVATION = "low_elevation"
        const val REJECTION_INVALID_TIME_UNCERTAINTY = "invalid_time_uncertainty"
        const val REJECTION_INVALID_PSEUDORANGE = "invalid_pseudorange"
        const val REJECTION_UNAVAILABLE_EPHEMERIS = "unavailable_ephemeris"
        const val REJECTION_UNHEALTHY_SATELLITE = "unhealthy_satellite"
        const val REJECTION_STALE_EPHEMERIS = "stale_ephemeris"
        const val REJECTION_MISSING_CORRECTION = "missing_correction"
        const val REJECTION_INSUFFICIENT_OBSERVATIONS = "insufficient_observations"

        private const val CONSTELLATION_GPS = 1
        private const val CONSTELLATION_QZSS = 4
        private const val CONSTELLATION_BEIDOU = 5
        private const val CONSTELLATION_GALILEO = 6
        private val SUPPORTED_CONSTELLATIONS = setOf(
            CONSTELLATION_GPS,
            CONSTELLATION_QZSS,
            CONSTELLATION_BEIDOU,
            CONSTELLATION_GALILEO,
        )
        private const val STATE_CODE_LOCK = 1
        private const val STATE_TOW_DECODED = 8
        private const val STATE_TOW_KNOWN = 16_384
        private const val WEEK_NANOS = 604_800_000_000_000L
        private const val HALF_WEEK_NANOS = WEEK_NANOS / 2
        private const val BEIDOU_TO_GPS_NANOS = 14_000_000_000L
        private const val SPEED_OF_LIGHT_METERS_PER_NANOSECOND = 0.299792458
        private const val MIN_FREQUENCY_HZ = 1.1e9
        private const val MAX_FREQUENCY_HZ = 1.7e9
        private const val MIN_CN0_DBHZ = 18.0
        private const val MIN_ELEVATION_DEGREES = 5.0
        private const val MAX_TRANSMIT_TIME_UNCERTAINTY_NANOS = 334L
        private const val MIN_PSEUDORANGE_METERS = 1.0e6
        private const val MAX_PSEUDORANGE_METERS = 1.0e8
    }
}
