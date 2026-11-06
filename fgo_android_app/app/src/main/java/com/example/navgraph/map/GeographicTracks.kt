package com.example.navgraph.map

import com.example.navgraph.estimator.Coordinates

enum class TrackTimeBase { MONOTONIC_NANOS, DATASET_MILLIS }

data class GeographicSample(
    val source: String,
    val timestamp: Long,
    val timeBase: TrackTimeBase,
    val latitudeDegrees: Double,
    val longitudeDegrees: Double,
    val uncertaintyMeters: Double? = null,
    val valid: Boolean = true,
    val breakBefore: Boolean = false,
) {
    val usable: Boolean get() = valid && latitudeDegrees.isFinite() &&
        longitudeDegrees.isFinite() && latitudeDegrees in -90.0..90.0 &&
        longitudeDegrees in -180.0..180.0
    fun stale(now: Long): Boolean = now - timestamp > gapLimit(timeBase)
}

fun gapLimit(base: TrackTimeBase): Long = if (base == TrackTimeBase.MONOTONIC_NANOS)
    5_000_000_000L else 5_000L

fun geographicFromEcef(source: String, timestampMillis: Long, ecefMeters: DoubleArray,
    valid: Boolean = true): GeographicSample {
    val lla = Coordinates.ecefToLla(ecefMeters)
    return GeographicSample(source, timestampMillis, TrackTimeBase.DATASET_MILLIS,
        lla[0], lla[1], valid = valid)
}

/** Splits at invalid samples, source/time changes, gaps and antimeridian crossings. */
fun geographicSegments(samples: List<GeographicSample>): List<List<GeographicSample>> {
    val result = mutableListOf<MutableList<GeographicSample>>()
    var previous: GeographicSample? = null
    samples.forEach { sample ->
        if (!sample.usable) { previous = null; return@forEach }
        val before = previous
        if (before == null || sample.breakBefore || sample.source != before.source ||
            sample.timeBase != before.timeBase || sample.timestamp <= before.timestamp ||
            sample.timestamp - before.timestamp > gapLimit(sample.timeBase) ||
            kotlin.math.abs(sample.longitudeDegrees - before.longitudeDegrees) > 180.0) {
            result.add(mutableListOf())
        }
        result.last().add(sample)
        previous = sample
    }
    return result
}

/** App scoped, display only. Logs retain all original samples. */
class GeographicHistory(private val limitPerSource: Int = 20_000) {
    init { require(limitPerSource >= 4) }
    private val tracks = linkedMapOf<String, MutableList<GeographicSample>>()
    private val pendingBreaks = mutableSetOf<String>()
    @Synchronized fun reset() { tracks.clear(); pendingBreaks.clear() }
    @Synchronized fun append(sample: GeographicSample) {
        val track = tracks.getOrPut(sample.source) { mutableListOf() }
        val previous = track.lastOrNull()
        if (previous?.timestamp == sample.timestamp && previous.timeBase == sample.timeBase) {
            if (sample.breakBefore) pendingBreaks.add(sample.source)
            return
        }
        val pendingBreak = pendingBreaks.remove(sample.source)
        track.add(sample.copy(breakBefore = sample.breakBefore || pendingBreak ||
            (previous != null && (sample.timestamp <= previous.timestamp ||
                sample.timeBase != previous.timeBase))))
        if (track.size > limitPerSource) {
            // Decimate interiors while explicitly retaining every surviving segment boundary.
            val segments = geographicSegments(track)
            val reduced = segments.flatMap { segment ->
                segment.filterIndexed { index, _ -> index == 0 || index == segment.lastIndex || index % 2 == 0 }
                    .mapIndexed { index, item -> item.copy(breakBefore = index == 0) }
            }
            track.clear()
            // Pathological discontinuity streams cannot retain all boundaries within a fixed cap.
            track.addAll(reduced.takeLast(limitPerSource).mapIndexed { i, s ->
                if (i == 0) s.copy(breakBefore = true) else s
            })
        }
    }
    @Synchronized fun snapshot(): Map<String, List<GeographicSample>> = tracks.mapValues { it.value.toList() }
}

object LiveMapSession { val history = GeographicHistory() }

fun replayThrough(tracks: List<GeographicSample>, timestampMillis: Long): List<GeographicSample> =
    tracks.filter { it.timeBase == TrackTimeBase.DATASET_MILLIS && it.timestamp <= timestampMillis }
