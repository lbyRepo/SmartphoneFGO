package com.example.navgraph.preprocessing

import com.example.navgraph.live.RawGnssNavigationMessageSample
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.sin
import kotlin.math.sqrt

data class GpsEphemerisProviderHealth(
    val receivedMessages: Long,
    val acceptedSubframes: Long,
    val rejectedMessages: Long,
    val completeEphemerides: Int,
    val lastRejectionReason: String,
)

interface NavigationMessageIngestor {
    fun ingest(message: RawGnssNavigationMessageSample)
}

interface EphemerisHealthSource {
    fun ephemerisHealth(): GpsEphemerisProviderHealth
}

data class GpsL1CaEphemeris(
    val svid: Int,
    val weekModulo1024: Int,
    val uraIndex: Int,
    val svHealth: Int,
    val iodc: Int,
    val iode: Int,
    val tgdSeconds: Double,
    val tocSeconds: Double,
    val af0Seconds: Double,
    val af1SecondsPerSecond: Double,
    val af2SecondsPerSecondSquared: Double,
    val crsMeters: Double,
    val deltaNRadiansPerSecond: Double,
    val m0Radians: Double,
    val cucRadians: Double,
    val eccentricity: Double,
    val cusRadians: Double,
    val sqrtSemiMajorAxisMetersSqrt: Double,
    val toeSeconds: Double,
    val cicRadians: Double,
    val omega0Radians: Double,
    val cisRadians: Double,
    val i0Radians: Double,
    val crcMeters: Double,
    val omegaRadians: Double,
    val omegaDotRadiansPerSecond: Double,
    val iDotRadiansPerSecond: Double,
)

class GpsL1CaEphemerisProvider(
    private val maximumAgeSeconds: Double = 14_400.0,
) : SatelliteStateProvider, NavigationMessageIngestor, EphemerisHealthSource {
    private val partial = mutableMapOf<Int, PartialEphemeris>()
    private val complete = mutableMapOf<Int, GpsL1CaEphemeris>()
    private var receivedMessages = 0L
    private var acceptedSubframes = 0L
    private var rejectedMessages = 0L
    private var lastRejectionReason = "none"

    init {
        require(maximumAgeSeconds > 0.0)
    }

    @Synchronized
    override fun ingest(message: RawGnssNavigationMessageSample) {
        receivedMessages++
        val rejection = validate(message)
        if (rejection != null) {
            rejectedMessages++
            lastRejectionReason = rejection
            return
        }
        val decoded = runCatching {
            GpsL1CaNavigationDecoder.decode(message.submessageId, message.data)
        }.getOrElse {
            rejectedMessages++
            lastRejectionReason = "decode_failure"
            return
        }
        val state = partial.getOrPut(message.svid) { PartialEphemeris() }
        when (decoded) {
            is GpsL1CaNavigationDecoder.Subframe1 -> state.subframe1 =
                TimedSubframe(decoded, message.elapsedRealtimeNanos)
            is GpsL1CaNavigationDecoder.Subframe2 -> state.subframe2 =
                TimedSubframe(decoded, message.elapsedRealtimeNanos)
            is GpsL1CaNavigationDecoder.Subframe3 -> state.subframe3 =
                TimedSubframe(decoded, message.elapsedRealtimeNanos)
        }
        acceptedSubframes++
        assemble(message.svid, state)?.let { complete[message.svid] = it }
    }

    @Synchronized
    override fun stateAt(signal: SatelliteSignalKey, transmitTimeNanos: Long): SatelliteStateResult {
        if (signal.constellation != CONSTELLATION_GPS) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.UNSUPPORTED_CONSTELLATION)
        }
        val frequency = signal.carrierFrequencyHz
        if (frequency != null && abs(frequency - GPS_L1_FREQUENCY_HZ) > L1_FREQUENCY_TOLERANCE_HZ) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.UNSUPPORTED_SIGNAL)
        }
        if (signal.codeType.isNotBlank() && !signal.codeType.equals("C", ignoreCase = true)) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.UNSUPPORTED_SIGNAL)
        }
        if (transmitTimeNanos <= 0L) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.INVALID_TIME)
        }
        val ephemeris = complete[signal.svid]
            ?: return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.UNAVAILABLE_EPHEMERIS)
        if (ephemeris.svHealth != 0) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.UNHEALTHY_SATELLITE)
        }
        val accuracyMeters = uraAccuracyMeters(ephemeris.uraIndex)
        if (!accuracyMeters.isFinite()) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.UNHEALTHY_SATELLITE)
        }
        val evaluated = GpsBroadcastOrbit.evaluate(ephemeris, transmitTimeNanos)
            ?: return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.INVALID_TIME)
        if (evaluated.sourceAgeSeconds > maximumAgeSeconds) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.STALE_EPHEMERIS)
        }
        return SatelliteStateResult.Available(SatelliteState(
            transmitTimeNanos = transmitTimeNanos,
            positionEcefMeters = evaluated.positionEcefMeters,
            velocityEcefMetersPerSecond = evaluated.velocityEcefMetersPerSecond,
            clockBiasMeters = evaluated.clockBiasMeters,
            clockDriftMetersPerSecond = evaluated.clockDriftMetersPerSecond,
            health = SatelliteHealth.HEALTHY,
            sourceAgeSeconds = evaluated.sourceAgeSeconds,
            accuracyMeters = accuracyMeters,
        ))
    }

    @Synchronized
    override fun ephemerisHealth(): GpsEphemerisProviderHealth = GpsEphemerisProviderHealth(
        receivedMessages = receivedMessages,
        acceptedSubframes = acceptedSubframes,
        rejectedMessages = rejectedMessages,
        completeEphemerides = complete.size,
        lastRejectionReason = lastRejectionReason,
    )

    private fun validate(message: RawGnssNavigationMessageSample): String? = when {
        message.messageType != TYPE_GPS_L1_CA -> "unsupported_message_type"
        message.svid !in 1..32 -> "invalid_svid"
        message.status !in setOf(STATUS_PARITY_PASSED, STATUS_PARITY_REBUILT) -> "parity_not_verified"
        message.submessageId !in 1..3 -> "unsupported_subframe"
        message.data.size != GPS_L1_CA_BYTES -> "invalid_length"
        GpsL1CaNavigationDecoder.embeddedSubframeId(message.data) != message.submessageId ->
            "subframe_id_mismatch"
        else -> null
    }

    private fun assemble(svid: Int, partial: PartialEphemeris): GpsL1CaEphemeris? {
        val storedOne = partial.subframe1 ?: return null
        val storedTwo = partial.subframe2 ?: return null
        val storedThree = partial.subframe3 ?: return null
        val times = longArrayOf(
            storedOne.elapsedRealtimeNanos,
            storedTwo.elapsedRealtimeNanos,
            storedThree.elapsedRealtimeNanos,
        )
        if (times.max() - times.min() > MAX_SUBFRAME_SPAN_NANOS) return null
        val one = storedOne.value
        val two = storedTwo.value
        val three = storedThree.value
        if (two.iode != three.iode || (one.iodc and 0xFF) != two.iode) return null
        return GpsL1CaEphemeris(
            svid = svid,
            weekModulo1024 = one.weekModulo1024,
            uraIndex = one.uraIndex,
            svHealth = one.svHealth,
            iodc = one.iodc,
            iode = two.iode,
            tgdSeconds = one.tgdSeconds,
            tocSeconds = one.tocSeconds,
            af0Seconds = one.af0Seconds,
            af1SecondsPerSecond = one.af1SecondsPerSecond,
            af2SecondsPerSecondSquared = one.af2SecondsPerSecondSquared,
            crsMeters = two.crsMeters,
            deltaNRadiansPerSecond = two.deltaNRadiansPerSecond,
            m0Radians = two.m0Radians,
            cucRadians = two.cucRadians,
            eccentricity = two.eccentricity,
            cusRadians = two.cusRadians,
            sqrtSemiMajorAxisMetersSqrt = two.sqrtSemiMajorAxisMetersSqrt,
            toeSeconds = two.toeSeconds,
            cicRadians = three.cicRadians,
            omega0Radians = three.omega0Radians,
            cisRadians = three.cisRadians,
            i0Radians = three.i0Radians,
            crcMeters = three.crcMeters,
            omegaRadians = three.omegaRadians,
            omegaDotRadiansPerSecond = three.omegaDotRadiansPerSecond,
            iDotRadiansPerSecond = three.iDotRadiansPerSecond,
        )
    }

    private fun uraAccuracyMeters(index: Int): Double = when (index) {
        1 -> 2.8
        3 -> 5.7
        5 -> 11.3
        in 0..5 -> Math.scalb(1.0, 1 + index / 2)
        in 6..14 -> Math.scalb(1.0, index - 2)
        else -> Double.NaN
    }

    private data class PartialEphemeris(
        var subframe1: TimedSubframe<GpsL1CaNavigationDecoder.Subframe1>? = null,
        var subframe2: TimedSubframe<GpsL1CaNavigationDecoder.Subframe2>? = null,
        var subframe3: TimedSubframe<GpsL1CaNavigationDecoder.Subframe3>? = null,
    )

    private data class TimedSubframe<T>(val value: T, val elapsedRealtimeNanos: Long)

    private companion object {
        const val CONSTELLATION_GPS = 1
        const val TYPE_GPS_L1_CA = 257
        const val STATUS_PARITY_PASSED = 1
        const val STATUS_PARITY_REBUILT = 2
        const val GPS_L1_CA_BYTES = 40
        const val GPS_L1_FREQUENCY_HZ = 1_575_420_000.0
        const val L1_FREQUENCY_TOLERANCE_HZ = 5_000_000.0
        const val MAX_SUBFRAME_SPAN_NANOS = 120_000_000_000L
    }
}

internal object GpsL1CaNavigationDecoder {
    sealed interface DecodedSubframe

    data class Subframe1(
        val weekModulo1024: Int,
        val uraIndex: Int,
        val svHealth: Int,
        val iodc: Int,
        val tgdSeconds: Double,
        val tocSeconds: Double,
        val af0Seconds: Double,
        val af1SecondsPerSecond: Double,
        val af2SecondsPerSecondSquared: Double,
    ) : DecodedSubframe

    data class Subframe2(
        val iode: Int,
        val crsMeters: Double,
        val deltaNRadiansPerSecond: Double,
        val m0Radians: Double,
        val cucRadians: Double,
        val eccentricity: Double,
        val cusRadians: Double,
        val sqrtSemiMajorAxisMetersSqrt: Double,
        val toeSeconds: Double,
    ) : DecodedSubframe

    data class Subframe3(
        val cicRadians: Double,
        val omega0Radians: Double,
        val cisRadians: Double,
        val i0Radians: Double,
        val crcMeters: Double,
        val omegaRadians: Double,
        val omegaDotRadiansPerSecond: Double,
        val iDotRadiansPerSecond: Double,
        val iode: Int,
    ) : DecodedSubframe

    fun decode(subframeId: Int, data: ByteArray): DecodedSubframe {
        require(data.size == 40)
        require(embeddedSubframeId(data) == subframeId)
        return when (subframeId) {
            1 -> decodeSubframe1(data)
            2 -> decodeSubframe2(data)
            3 -> decodeSubframe3(data)
            else -> error("Unsupported GPS L1 C/A subframe: $subframeId")
        }
    }

    fun embeddedSubframeId(data: ByteArray): Int =
        if (data.size == 40) unsigned(data, 49, 3).toInt() else -1

    private fun decodeSubframe1(data: ByteArray): Subframe1 {
        val iodc = (unsigned(data, 82, 2).toInt() shl 8) or unsigned(data, 210, 8).toInt()
        return Subframe1(
            weekModulo1024 = unsigned(data, 60, 10).toInt(),
            uraIndex = unsigned(data, 72, 4).toInt(),
            svHealth = unsigned(data, 76, 6).toInt(),
            iodc = iodc,
            tgdSeconds = signed(data, 196, 8) * pow2(-31),
            tocSeconds = unsigned(data, 218, 16) * pow2(4),
            af0Seconds = signed(data, 270, 22) * pow2(-31),
            af1SecondsPerSecond = signed(data, 248, 16) * pow2(-43),
            af2SecondsPerSecondSquared = signed(data, 240, 8) * pow2(-55),
        )
    }

    private fun decodeSubframe2(data: ByteArray): Subframe2 = Subframe2(
        iode = unsigned(data, 60, 8).toInt(),
        crsMeters = signed(data, 68, 16) * pow2(-5),
        deltaNRadiansPerSecond = signed(data, 90, 16) * pow2(-43) * PI,
        m0Radians = signedSplit32(data, 106, 120) * pow2(-31) * PI,
        cucRadians = signed(data, 150, 16) * pow2(-29),
        eccentricity = unsignedSplit32(data, 166, 180) * pow2(-33),
        cusRadians = signed(data, 210, 16) * pow2(-29),
        sqrtSemiMajorAxisMetersSqrt = unsignedSplit32(data, 226, 240) * pow2(-19),
        toeSeconds = unsigned(data, 270, 16) * pow2(4),
    )

    private fun decodeSubframe3(data: ByteArray): Subframe3 = Subframe3(
        cicRadians = signed(data, 60, 16) * pow2(-29),
        omega0Radians = signedSplit32(data, 76, 90) * pow2(-31) * PI,
        cisRadians = signed(data, 120, 16) * pow2(-29),
        i0Radians = signedSplit32(data, 136, 150) * pow2(-31) * PI,
        crcMeters = signed(data, 180, 16) * pow2(-5),
        omegaRadians = signedSplit32(data, 196, 210) * pow2(-31) * PI,
        omegaDotRadiansPerSecond = signed(data, 240, 24) * pow2(-43) * PI,
        iDotRadiansPerSecond = signed(data, 278, 14) * pow2(-43) * PI,
        iode = unsigned(data, 270, 8).toInt(),
    )

    private fun unsignedSplit32(data: ByteArray, index8: Int, index24: Int): Long =
        (unsigned(data, index8, 8) shl 24) or unsigned(data, index24, 24)

    private fun signedSplit32(data: ByteArray, index8: Int, index24: Int): Long {
        val raw = unsignedSplit32(data, index8, index24)
        return if (raw and 0x8000_0000L != 0L) raw - 0x1_0000_0000L else raw
    }

    private fun signed(data: ByteArray, index: Int, length: Int): Long {
        val raw = unsigned(data, index, length)
        val sign = 1L shl (length - 1)
        return if (raw and sign != 0L) raw - (1L shl length) else raw
    }

    private fun unsigned(data: ByteArray, index: Int, length: Int): Long {
        require(index >= 0 && length in 1..32 && index + length <= 300)
        var result = 0L
        repeat(length) { offset ->
            val logicalIndex = index + offset
            val wordIndex = logicalIndex / 30
            val physicalIndex = logicalIndex + (wordIndex + 1) * 2
            val byteValue = data[physicalIndex / 8].toInt() and 0xFF
            val bit = (byteValue ushr (7 - physicalIndex % 8)) and 1
            result = (result shl 1) or bit.toLong()
        }
        return result
    }

    private fun pow2(exponent: Int): Double = Math.scalb(1.0, exponent)
}

internal object GpsBroadcastOrbit {
    data class EvaluatedState(
        val positionEcefMeters: DoubleArray,
        val velocityEcefMetersPerSecond: DoubleArray,
        val clockBiasMeters: Double,
        val clockDriftMetersPerSecond: Double,
        val sourceAgeSeconds: Double,
    )

    fun evaluate(ephemeris: GpsL1CaEphemeris, gpsTimeNanos: Long): EvaluatedState? {
        if (gpsTimeNanos <= 0L || ephemeris.eccentricity !in 0.0..<1.0 ||
            ephemeris.sqrtSemiMajorAxisMetersSqrt !in 4_000.0..6_000.0
        ) return null
        val gpsSeconds = gpsTimeNanos * 1.0e-9
        val center = positionAndClock(ephemeris, gpsSeconds) ?: return null
        val plus = positionAndClock(ephemeris, gpsSeconds + 0.5) ?: return null
        val minus = positionAndClock(ephemeris, gpsSeconds - 0.5) ?: return null
        return EvaluatedState(
            positionEcefMeters = center.position,
            velocityEcefMetersPerSecond = DoubleArray(3) { plus.position[it] - minus.position[it] },
            clockBiasMeters = center.clockMeters,
            clockDriftMetersPerSecond = plus.clockMeters - minus.clockMeters,
            sourceAgeSeconds = abs(center.tkSeconds),
        )
    }

    private fun positionAndClock(ephemeris: GpsL1CaEphemeris, gpsSeconds: Double): PositionClock? {
        val queryWeek = floor(gpsSeconds / SECONDS_PER_WEEK).toInt()
        val queryTow = gpsSeconds - queryWeek * SECONDS_PER_WEEK
        val ephemerisWeek = resolveWeek(ephemeris.weekModulo1024, queryWeek)
        val weekOffset = (queryWeek - ephemerisWeek) * SECONDS_PER_WEEK
        val tc = normalizeWeek(weekOffset + queryTow - ephemeris.tocSeconds)
        val polynomial = ephemeris.af0Seconds + ephemeris.af1SecondsPerSecond * tc +
            ephemeris.af2SecondsPerSecondSquared * tc * tc - ephemeris.tgdSeconds
        val semiMajorAxis = ephemeris.sqrtSemiMajorAxisMetersSqrt *
            ephemeris.sqrtSemiMajorAxisMetersSqrt
        val meanMotion = sqrt(EARTH_GRAVITATIONAL_PARAMETER / (semiMajorAxis * semiMajorAxis * semiMajorAxis)) +
            ephemeris.deltaNRadiansPerSecond
        var clockSeconds = polynomial
        var eccentricAnomaly = 0.0
        var tk = 0.0
        repeat(10) {
            tk = normalizeWeek(weekOffset + queryTow - ephemeris.toeSeconds - clockSeconds)
            val meanAnomaly = ephemeris.m0Radians + meanMotion * tk
            eccentricAnomaly = solveKepler(meanAnomaly, ephemeris.eccentricity) ?: return null
            val relativistic = RELATIVISTIC_CONSTANT * ephemeris.eccentricity *
                ephemeris.sqrtSemiMajorAxisMetersSqrt * sin(eccentricAnomaly)
            val updated = polynomial + relativistic
            if (abs(updated - clockSeconds) < 1.0e-12) {
                clockSeconds = updated
                return@repeat
            }
            clockSeconds = updated
        }
        tk = normalizeWeek(weekOffset + queryTow - ephemeris.toeSeconds - clockSeconds)
        val trueAnomaly = atan2(
            sqrt(1.0 - ephemeris.eccentricity * ephemeris.eccentricity) * sin(eccentricAnomaly),
            cos(eccentricAnomaly) - ephemeris.eccentricity,
        )
        val phi = trueAnomaly + ephemeris.omegaRadians
        val twicePhi = 2.0 * phi
        val argument = phi + ephemeris.cusRadians * sin(twicePhi) +
            ephemeris.cucRadians * cos(twicePhi)
        val radius = semiMajorAxis * (1.0 - ephemeris.eccentricity * cos(eccentricAnomaly)) +
            ephemeris.crsMeters * sin(twicePhi) + ephemeris.crcMeters * cos(twicePhi)
        val inclination = ephemeris.i0Radians + ephemeris.iDotRadiansPerSecond * tk +
            ephemeris.cisRadians * sin(twicePhi) + ephemeris.cicRadians * cos(twicePhi)
        val orbitalX = radius * cos(argument)
        val orbitalY = radius * sin(argument)
        val ascendingNode = ephemeris.omega0Radians +
            (ephemeris.omegaDotRadiansPerSecond - EARTH_ROTATION_RATE) * tk -
            EARTH_ROTATION_RATE * ephemeris.toeSeconds
        val position = doubleArrayOf(
            orbitalX * cos(ascendingNode) - orbitalY * cos(inclination) * sin(ascendingNode),
            orbitalX * sin(ascendingNode) + orbitalY * cos(inclination) * cos(ascendingNode),
            orbitalY * sin(inclination),
        )
        return PositionClock(position, clockSeconds * SPEED_OF_LIGHT_METERS_PER_SECOND, tk)
    }

    private fun solveKepler(meanAnomaly: Double, eccentricity: Double): Double? {
        var eccentricAnomaly = meanAnomaly
        repeat(20) {
            val next = meanAnomaly + eccentricity * sin(eccentricAnomaly)
            if (abs(next - eccentricAnomaly) < 1.0e-13) return next
            eccentricAnomaly = next
        }
        return null
    }

    private fun resolveWeek(moduloWeek: Int, queryWeek: Int): Int {
        var candidate = queryWeek - Math.floorMod(queryWeek - moduloWeek, 1024)
        if (queryWeek - candidate > 512) candidate += 1024
        return candidate
    }

    private fun normalizeWeek(seconds: Double): Double = seconds -
        floor((seconds + HALF_WEEK_SECONDS) / SECONDS_PER_WEEK) * SECONDS_PER_WEEK

    private data class PositionClock(
        val position: DoubleArray,
        val clockMeters: Double,
        val tkSeconds: Double,
    )

    private const val SECONDS_PER_WEEK = 604_800.0
    private const val HALF_WEEK_SECONDS = 302_400.0
    private const val EARTH_GRAVITATIONAL_PARAMETER = 3.986005e14
    private const val EARTH_ROTATION_RATE = 7.2921151467e-5
    private const val RELATIVISTIC_CONSTANT = -4.442807633e-10
    private const val SPEED_OF_LIGHT_METERS_PER_SECOND = 299_792_458.0
}
