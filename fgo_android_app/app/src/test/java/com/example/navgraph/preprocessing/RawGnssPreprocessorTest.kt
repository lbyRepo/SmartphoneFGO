package com.example.navgraph.preprocessing

import com.example.navgraph.estimator.GnssWlsSolver
import com.example.navgraph.estimator.Coordinates
import com.example.navgraph.estimator.EnuFrame
import com.example.navgraph.estimator.SyntheticGnssInsDatasetFactory
import com.example.navgraph.live.RawGnssClockSample
import com.example.navgraph.live.RawGnssEpochSample
import com.example.navgraph.live.RawGnssObservationSample
import kotlin.math.roundToLong
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class RawGnssPreprocessorTest {
    @Test
    fun rawClockAndMeasurementsBecomeSolverReadyEpoch() {
        val source = SyntheticGnssInsDatasetFactory.create().gnssEpochs.first()
        val receiveGpsNanos = 2L * WEEK_NANOS + 100_000_000_000L
        val clockTimeNanos = 500_000_000_000L
        val fullBiasNanos = clockTimeNanos - receiveGpsNanos
        val snapshots = mutableListOf<SatelliteStateSnapshot>()
        val raw = source.observations.map { observation ->
            val timeOffsetNanos = if (observation.svid == 1) 250_000.0 else 0.0
            val travelNanos = (observation.correctedPseudorangeMeters / C_METERS_PER_NANOSECOND).roundToLong()
            val transmitTowNanos = Math.floorMod(receiveGpsNanos, WEEK_NANOS) +
                timeOffsetNanos.toLong() - travelNanos
            val key = SatelliteSignalKey(1, observation.svid, 1_575_420_000.0, "C")
            snapshots += SatelliteStateSnapshot(
                signal = key,
                referenceTimeNanos = 2L * WEEK_NANOS + transmitTowNanos,
                positionEcefMeters = observation.satellitePositionMeters,
                velocityEcefMetersPerSecond = observation.satelliteVelocityMetersPerSecond,
                clockBiasMeters = 0.0,
                clockDriftMetersPerSecond = 0.0,
            )
            RawGnssObservationSample(
                constellation = 1,
                svid = observation.svid,
                state = STATE_CODE_LOCK or STATE_TOW_DECODED,
                timeOffsetNanos = timeOffsetNanos,
                receivedSvTimeNanos = transmitTowNanos,
                receivedSvTimeUncertaintyNanos = 5L,
                cn0DbHz = observation.cn0DbHz,
                pseudorangeRateMetersPerSecond = observation.correctedRangeRateMetersPerSecond,
                pseudorangeRateUncertaintyMetersPerSecond = observation.rangeRateSigmaMetersPerSecond,
                carrierFrequencyHz = 1_575_420_000.0,
                codeType = "C",
                fullInterSignalBiasNanos = null,
                satelliteInterSignalBiasNanos = null,
            )
        }
        val preprocessor = RawGnssPreprocessor(
            SnapshotSatelliteStateProvider(snapshots, maximumAgeSeconds = 1.0),
            zeroCorrections,
        )
        val result = preprocessor.consume(epoch(clockTimeNanos, fullBiasNanos, raw))

        val clean = assertNotNull(result.epoch)
        assertEquals(source.observations.size, result.acceptedObservationCount)
        assertTrue(result.rejectionCounts.isEmpty())
        assertEquals(123_000_000_000L, clean.elapsedRealtimeNanos)
        clean.observations.zip(source.observations).forEach { (actual, expected) ->
            assertTrue(kotlin.math.abs(actual.correctedPseudorangeMeters - expected.correctedPseudorangeMeters) < 0.2)
        }
        val estimate = GnssWlsSolver().solve(listOf(clean)).single()
        assertTrue(estimate.converged)
    }

    @Test
    fun unavailableEphemerisHasStableRejectionReason() {
        val input = epoch(
            clockTimeNanos = 500_000_000_000L,
            fullBiasNanos = 500_000_000_000L - (2L * WEEK_NANOS + 100_000_000_000L),
            observations = List(4) { index -> validRaw(index + 1) },
        )
        val result = RawGnssPreprocessor(
            UnavailableSatelliteStateProvider,
            zeroCorrections,
        ).consume(input)

        assertEquals(null, result.epoch)
        assertEquals(4, result.rejectionCounts[RawGnssPreprocessor.REJECTION_UNAVAILABLE_EPHEMERIS])
        assertEquals(1, result.rejectionCounts[RawGnssPreprocessor.REJECTION_INSUFFICIENT_OBSERVATIONS])
    }

    @Test
    fun invalidMeasurementsAreRejectedBeforeEphemerisLookup() {
        val observations = listOf(
            validRaw(1).copy(cn0DbHz = 12.0),
            validRaw(2).copy(state = STATE_CODE_LOCK),
            validRaw(3).copy(carrierFrequencyHz = 900_000_000.0),
            validRaw(4).copy(receivedSvTimeUncertaintyNanos = 1_000L),
        )
        val result = RawGnssPreprocessor(
            UnavailableSatelliteStateProvider,
            zeroCorrections,
        ).consume(epoch(
            500_000_000_000L,
            500_000_000_000L - (2L * WEEK_NANOS + 100_000_000_000L),
            observations,
        ))

        assertEquals(1, result.rejectionCounts[RawGnssPreprocessor.REJECTION_LOW_SIGNAL])
        assertEquals(1, result.rejectionCounts[RawGnssPreprocessor.REJECTION_INVALID_TRACKING_STATE])
        assertEquals(1, result.rejectionCounts[RawGnssPreprocessor.REJECTION_UNSUPPORTED_FREQUENCY])
        assertEquals(1, result.rejectionCounts[RawGnssPreprocessor.REJECTION_INVALID_TIME_UNCERTAINTY])
    }

    @Test
    fun snapshotProviderPropagatesAndRejectsStaleState() {
        val key = SatelliteSignalKey(1, 3, 1_575_420_000.0, "C")
        val reference = 1_000_000_000_000L
        val provider = SnapshotSatelliteStateProvider(listOf(SatelliteStateSnapshot(
            signal = key,
            referenceTimeNanos = reference,
            positionEcefMeters = doubleArrayOf(20_000_000.0, 0.0, 0.0),
            velocityEcefMetersPerSecond = doubleArrayOf(0.0, 3_000.0, 0.0),
            clockBiasMeters = 10.0,
            clockDriftMetersPerSecond = 0.2,
        )), maximumAgeSeconds = 10.0)

        val available = provider.stateAt(key, reference + 2_000_000_000L) as SatelliteStateResult.Available
        assertEquals(6_000.0, available.state.positionEcefMeters[1], 1.0e-9)
        assertEquals(10.4, available.state.clockBiasMeters, 1.0e-9)
        val stale = provider.stateAt(key, reference + 11_000_000_000L) as SatelliteStateResult.Unavailable
        assertEquals(SatelliteStateFailureReason.STALE_EPHEMERIS, stale.reason)
    }

    @Test
    fun satelliteBelowElevationMaskIsRejectedWhenReferenceExists() {
        val receiver = Coordinates.llaToEcef(1.3521, 103.8198, 20.0)
        val frame = EnuFrame(receiver)
        val downEcef = frame.enuVectorToEcef(doubleArrayOf(0.0, 0.0, -21_000_000.0))
        val satellite = DoubleArray(3) { receiver[it] + downEcef[it] }
        val raw = validRaw(1)
        val key = SatelliteSignalKey(1, 1, raw.carrierFrequencyHz, raw.codeType)
        val provider = SnapshotSatelliteStateProvider(listOf(SatelliteStateSnapshot(
            signal = key,
            referenceTimeNanos = 2L * WEEK_NANOS + raw.receivedSvTimeNanos,
            positionEcefMeters = satellite,
            velocityEcefMetersPerSecond = DoubleArray(3),
            clockBiasMeters = 0.0,
            clockDriftMetersPerSecond = 0.0,
        )))
        val preprocessor = RawGnssPreprocessor(provider, zeroCorrections)
        preprocessor.updateApproximateReceiver(1.3521, 103.8198, 20.0)
        val result = preprocessor.consume(epoch(
            500_000_000_000L,
            500_000_000_000L - (2L * WEEK_NANOS + 100_000_000_000L),
            listOf(raw),
        ))

        assertEquals(1, result.rejectionCounts[RawGnssPreprocessor.REJECTION_LOW_ELEVATION])
    }

    private fun epoch(
        clockTimeNanos: Long,
        fullBiasNanos: Long,
        observations: List<RawGnssObservationSample>,
    ) = RawGnssEpochSample(
        elapsedRealtimeNanos = 123_000_000_000L,
        clock = RawGnssClockSample(
            elapsedRealtimeNanos = 123_000_000_000L,
            callbackElapsedRealtimeNanos = 123_001_000_000L,
            timeNanos = clockTimeNanos,
            hardwareClockDiscontinuityCount = 4,
            fullBiasNanos = fullBiasNanos,
            biasNanos = 0.0,
            biasUncertaintyNanos = 1.0,
            driftNanosPerSecond = 0.0,
            driftUncertaintyNanosPerSecond = 1.0,
        ),
        observations = observations,
    )

    private fun validRaw(svid: Int) = RawGnssObservationSample(
        constellation = 1,
        svid = svid,
        state = STATE_CODE_LOCK or STATE_TOW_DECODED,
        timeOffsetNanos = 0.0,
        receivedSvTimeNanos = 99_930_000_000L,
        receivedSvTimeUncertaintyNanos = 10L,
        cn0DbHz = 35.0,
        pseudorangeRateMetersPerSecond = 0.0,
        pseudorangeRateUncertaintyMetersPerSecond = 0.2,
        carrierFrequencyHz = 1_575_420_000.0,
        codeType = "C",
        fullInterSignalBiasNanos = null,
        satelliteInterSignalBiasNanos = null,
    )

    private val zeroCorrections = CorrectionProvider { _, _, _ ->
        CorrectionResult.Available(ObservationCorrections(
            earthRotationMeters = 0.0,
            ionosphereMeters = 0.0,
            troposphereMeters = 0.0,
            interSignalBiasMeters = 0.0,
            uncertaintyMeters = 1.0,
            atmosphericModelComplete = true,
        ))
    }

    private companion object {
        const val WEEK_NANOS = 604_800_000_000_000L
        const val C_METERS_PER_NANOSECOND = 0.299792458
        const val STATE_CODE_LOCK = 1
        const val STATE_TOW_DECODED = 8
    }
}
