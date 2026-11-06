package com.example.navgraph.preprocessing

import com.example.navgraph.live.RawGnssNavigationMessageSample
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.roundToLong
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertIs
import kotlin.test.assertTrue

class GpsL1CaEphemerisProviderTest {
    @Test
    fun paddedAndroidWordsDecodeWithSignedScales() {
        val one = assertIs<GpsL1CaNavigationDecoder.Subframe1>(
            GpsL1CaNavigationDecoder.decode(1, subframe1()),
        )
        val two = assertIs<GpsL1CaNavigationDecoder.Subframe2>(
            GpsL1CaNavigationDecoder.decode(2, subframe2()),
        )
        val three = assertIs<GpsL1CaNavigationDecoder.Subframe3>(
            GpsL1CaNavigationDecoder.decode(3, subframe3()),
        )

        assertEquals(WEEK_MODULO, one.weekModulo1024)
        assertEquals(IODC, one.iodc)
        assertEquals(IODE, two.iode)
        assertEquals(IODE, three.iode)
        assertEquals(-10.0 * Math.scalb(1.0, -31), one.tgdSeconds, 1.0e-20)
        assertEquals(-1_000.0 * Math.scalb(1.0, -5), two.crsMeters, 1.0e-12)
        assertEquals(0.25 * PI, two.m0Radians, 1.0e-10)
        assertEquals(0.01, two.eccentricity, 2.0e-10)
        assertEquals(-0.375 * PI, three.omega0Radians, 1.0e-10)
        assertEquals(0.5 * PI, three.i0Radians, 1.0e-10)
        assertEquals(0.125 * PI, three.omegaRadians, 1.0e-10)
    }

    @Test
    fun matchingSubframesProduceUsableBroadcastState() {
        val provider = GpsL1CaEphemerisProvider()
        provider.ingest(message(1, subframe1()))
        provider.ingest(message(2, subframe2()))
        provider.ingest(message(3, subframe3()))

        assertEquals(1, provider.ephemerisHealth().completeEphemerides)
        val querySeconds = ABSOLUTE_WEEK * WEEK_SECONDS + TOE_SECONDS + 900.0
        val result = provider.stateAt(
            SatelliteSignalKey(1, SVID, 1_575_420_000.0, "C"),
            (querySeconds * 1.0e9).roundToLong(),
        )
        val state = assertIs<SatelliteStateResult.Available>(result).state
        val radius = norm(state.positionEcefMeters)
        val speed = norm(state.velocityEcefMetersPerSecond)

        assertTrue(radius in 25_000_000.0..28_000_000.0)
        assertTrue(speed in 1_000.0..5_000.0)
        assertTrue(state.clockBiasMeters.isFinite())
        assertTrue(abs(state.clockDriftMetersPerSecond) < 10.0)
        assertEquals(900.0, state.sourceAgeSeconds, 0.01)
        assertVectorEquals(
            doubleArrayOf(5_454_952.158_701, 3_376_364.600_164, 25_610_318.325_144),
            state.positionEcefMeters,
            1.0e-3,
        )
        assertVectorEquals(
            doubleArrayOf(-2_961.726_239, -2_383.408_329, 977.138_807),
            state.velocityEcefMetersPerSecond,
            1.0e-3,
        )
        assertEquals(-6_987.252_586, state.clockBiasMeters, 1.0e-6)
        assertEquals(-0.004_044_657, state.clockDriftMetersPerSecond, 1.0e-9)
        assertEquals(4.0, state.accuracyMeters, 1.0e-12)
    }

    @Test
    fun parityIssueMismatchStalenessAndL5AreExplicit() {
        val parityRejected = GpsL1CaEphemerisProvider()
        parityRejected.ingest(message(1, subframe1(), status = 0))
        assertEquals(1L, parityRejected.ephemerisHealth().rejectedMessages)
        assertEquals("parity_not_verified", parityRejected.ephemerisHealth().lastRejectionReason)

        val mismatched = GpsL1CaEphemerisProvider()
        mismatched.ingest(message(1, subframe1()))
        mismatched.ingest(message(2, subframe2()))
        mismatched.ingest(message(3, subframe3(iode = IODE + 1)))
        assertEquals(0, mismatched.ephemerisHealth().completeEphemerides)

        val incoherent = GpsL1CaEphemerisProvider()
        incoherent.ingest(message(1, subframe1(), elapsedRealtimeNanos = 1L))
        incoherent.ingest(message(2, subframe2(), elapsedRealtimeNanos = 2L))
        incoherent.ingest(message(3, subframe3(), elapsedRealtimeNanos = 121_000_000_001L))
        assertEquals(0, incoherent.ephemerisHealth().completeEphemerides)

        val provider = GpsL1CaEphemerisProvider(maximumAgeSeconds = 1_000.0)
        provider.ingest(message(1, subframe1()))
        provider.ingest(message(2, subframe2()))
        provider.ingest(message(3, subframe3()))
        val staleTime = ((ABSOLUTE_WEEK * WEEK_SECONDS + TOE_SECONDS + 2_000.0) * 1.0e9).roundToLong()
        val stale = assertIs<SatelliteStateResult.Unavailable>(provider.stateAt(
            SatelliteSignalKey(1, SVID, 1_575_420_000.0, "C"),
            staleTime,
        ))
        assertEquals(SatelliteStateFailureReason.STALE_EPHEMERIS, stale.reason)
        val l5 = assertIs<SatelliteStateResult.Unavailable>(provider.stateAt(
            SatelliteSignalKey(1, SVID, 1_176_450_000.0, "I"),
            staleTime,
        ))
        assertEquals(SatelliteStateFailureReason.UNSUPPORTED_SIGNAL, l5.reason)
    }

    private fun message(
        subframe: Int,
        data: ByteArray,
        status: Int = 1,
        elapsedRealtimeNanos: Long = subframe.toLong(),
    ) =
        RawGnssNavigationMessageSample(
            elapsedRealtimeNanos = elapsedRealtimeNanos,
            svid = SVID,
            messageType = 257,
            messageId = -1,
            submessageId = subframe,
            status = status,
            data = data,
        )

    private fun subframe1(): ByteArray = NavigationWordWriter(1).apply {
        unsigned(60, 10, WEEK_MODULO.toLong())
        unsigned(72, 4, 2)
        unsigned(76, 6, 0)
        unsigned(82, 2, (IODC ushr 8).toLong())
        signed(196, 8, -10)
        unsigned(210, 8, (IODC and 0xFF).toLong())
        unsigned(218, 16, (TOC_SECONDS / 16.0).roundToLong())
        signed(240, 8, -2)
        signed(248, 16, -100)
        signed(270, 22, -50_000)
    }.data

    private fun subframe2(): ByteArray = NavigationWordWriter(2).apply {
        unsigned(60, 8, IODE.toLong())
        signed(68, 16, -1_000)
        signed(90, 16, 100)
        signedSplit32(106, 120, 0x2000_0000)
        signed(150, 16, -200)
        unsignedSplit32(166, 180, (0.01 * Math.scalb(1.0, 33)).roundToLong())
        signed(210, 16, 300)
        unsignedSplit32(
            226,
            240,
            (5_153.795_477_5 * Math.scalb(1.0, 19)).roundToLong(),
        )
        unsigned(270, 16, (TOE_SECONDS / 16.0).roundToLong())
    }.data

    private fun subframe3(iode: Int = IODE): ByteArray = NavigationWordWriter(3).apply {
        signed(60, 16, 120)
        signedSplit32(76, 90, -0x3000_0000)
        signed(120, 16, -80)
        signedSplit32(136, 150, 0x4000_0000)
        signed(180, 16, 2_000)
        signedSplit32(196, 210, 0x1000_0000)
        signed(240, 24, -22_000)
        unsigned(270, 8, iode.toLong())
        signed(278, 14, 500)
    }.data

    private fun norm(vector: DoubleArray): Double = sqrt(vector.sumOf { it * it })

    private fun assertVectorEquals(expected: DoubleArray, actual: DoubleArray, tolerance: Double) {
        expected.indices.forEach { index -> assertEquals(expected[index], actual[index], tolerance) }
    }

    private class NavigationWordWriter(subframeId: Int) {
        val data = ByteArray(40)

        init {
            unsigned(49, 3, subframeId.toLong())
        }

        fun unsigned(index: Int, length: Int, value: Long) {
            require(length in 1..32)
            require(value >= 0L && (length == 32 || value < (1L shl length)))
            repeat(length) { offset ->
                val bit = ((value ushr (length - 1 - offset)) and 1L).toInt()
                val logical = index + offset
                val word = logical / 30
                val physical = logical + (word + 1) * 2
                val byteIndex = physical / 8
                val shift = 7 - physical % 8
                data[byteIndex] = (data[byteIndex].toInt() or (bit shl shift)).toByte()
            }
        }

        fun signed(index: Int, length: Int, value: Long) {
            val mask = (1L shl length) - 1L
            unsigned(index, length, value and mask)
        }

        fun unsignedSplit32(index8: Int, index24: Int, value: Long) {
            unsigned(index8, 8, (value ushr 24) and 0xFF)
            unsigned(index24, 24, value and 0xFF_FFFF)
        }

        fun signedSplit32(index8: Int, index24: Int, value: Long) =
            unsignedSplit32(index8, index24, value and 0xFFFF_FFFFL)
    }

    private companion object {
        const val SVID = 7
        const val IODE = 0x55
        const val IODC = 0x155
        const val WEEK_MODULO = 123
        const val ABSOLUTE_WEEK = 2_171
        const val WEEK_SECONDS = 604_800.0
        const val TOE_SECONDS = 320_000.0
        const val TOC_SECONDS = 320_000.0
    }
}
