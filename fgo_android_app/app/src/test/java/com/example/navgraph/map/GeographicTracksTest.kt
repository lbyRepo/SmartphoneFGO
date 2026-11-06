package com.example.navgraph.map

import com.example.navgraph.estimator.Coordinates
import org.junit.Test
import org.junit.Assert.*

class GeographicTracksTest {
    private fun sample(time: Long, source: String = "FGO", lon: Double = 103.8) =
        GeographicSample(source, time, TrackTimeBase.DATASET_MILLIS, 1.3, lon)

    @Test fun geographicConversionPreservesLatitudeAndLongitude() {
        val p = geographicFromEcef("WLS", 20, Coordinates.llaToEcef(1.3, 103.8, 24.0))
        assertEquals(1.3, p.latitudeDegrees, 1e-8)
        assertEquals(103.8, p.longitudeDegrees, 1e-8)
        assertEquals(TrackTimeBase.DATASET_MILLIS, p.timeBase)
    }
    @Test fun separatesSourcesAndDeduplicatesWithinSource() {
        val h = GeographicHistory()
        h.append(sample(100)); h.append(sample(100)); h.append(sample(100, "WLS"))
        assertEquals(2, h.snapshot().size)
        assertEquals(1, h.snapshot().getValue("FGO").size)
        h.reset(); assertTrue(h.snapshot().isEmpty())
    }
    @Test fun splitsInvalidSamplesGapsDiscontinuitiesAndAntimeridian() {
        val track = listOf(sample(0), sample(1000), sample(2000).copy(valid = false),
            sample(3000), sample(9000), sample(8000), sample(9000, lon = 179.9), sample(10000, lon = -179.9))
        assertEquals(listOf(2, 1, 1, 2, 1), geographicSegments(track).map { it.size })
        assertFalse(sample(0).stale(5000)); assertTrue(sample(0).stale(5001))
    }
    @Test fun boundRetainsEndpointsAndDoesNotJoinSegments() {
        val h = GeographicHistory(20)
        repeat(200) { h.append(sample(it * 100L)) }
        val track = h.snapshot().getValue("FGO")
        assertTrue(track.size <= 20)
        assertEquals(0L, track.first().timestamp)
        assertEquals(19900L, track.last().timestamp)
    }
    @Test fun clockBoundarySurvivesDuplicateStatusSample() {
        val h = GeographicHistory()
        h.append(sample(0)); h.append(sample(1000))
        h.append(sample(1000).copy(breakBefore = true))
        h.append(sample(2000))
        assertEquals(listOf(2, 1), geographicSegments(h.snapshot().getValue("FGO")).map { it.size })
    }
    @Test fun seekingNeverRevealsFutureOrLiveSamples() {
        val points = listOf(sample(0), sample(100), sample(200), sample(0).copy(timeBase = TrackTimeBase.MONOTONIC_NANOS))
        assertEquals(listOf(0L, 100L), replayThrough(points, 150).map { it.timestamp })
        assertEquals(1, replayThrough(points, 0).size)
    }
}
