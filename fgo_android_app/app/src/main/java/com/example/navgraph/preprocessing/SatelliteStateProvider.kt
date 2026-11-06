package com.example.navgraph.preprocessing

import kotlin.math.abs

data class SatelliteSignalKey(
    val constellation: Int,
    val svid: Int,
    val carrierFrequencyHz: Double?,
    val codeType: String,
)

enum class SatelliteHealth {
    HEALTHY,
    UNHEALTHY,
    UNKNOWN,
}

data class SatelliteState(
    val transmitTimeNanos: Long,
    val positionEcefMeters: DoubleArray,
    val velocityEcefMetersPerSecond: DoubleArray,
    val clockBiasMeters: Double,
    val clockDriftMetersPerSecond: Double,
    val health: SatelliteHealth,
    val sourceAgeSeconds: Double,
    val accuracyMeters: Double,
)

enum class SatelliteStateFailureReason {
    UNAVAILABLE_EPHEMERIS,
    UNHEALTHY_SATELLITE,
    STALE_EPHEMERIS,
    UNSUPPORTED_CONSTELLATION,
    UNSUPPORTED_SIGNAL,
    INVALID_TIME,
}

sealed interface SatelliteStateResult {
    data class Available(val state: SatelliteState) : SatelliteStateResult
    data class Unavailable(val reason: SatelliteStateFailureReason) : SatelliteStateResult
}

fun interface SatelliteStateProvider {
    fun stateAt(signal: SatelliteSignalKey, transmitTimeNanos: Long): SatelliteStateResult
}

object UnavailableSatelliteStateProvider : SatelliteStateProvider {
    override fun stateAt(signal: SatelliteSignalKey, transmitTimeNanos: Long): SatelliteStateResult =
        SatelliteStateResult.Unavailable(SatelliteStateFailureReason.UNAVAILABLE_EPHEMERIS)
}

data class SatelliteStateSnapshot(
    val signal: SatelliteSignalKey,
    val referenceTimeNanos: Long,
    val positionEcefMeters: DoubleArray,
    val velocityEcefMetersPerSecond: DoubleArray,
    val clockBiasMeters: Double,
    val clockDriftMetersPerSecond: Double,
    val health: SatelliteHealth = SatelliteHealth.HEALTHY,
    val accuracyMeters: Double = 2.4,
) {
    init {
        require(positionEcefMeters.size == 3 && positionEcefMeters.all(Double::isFinite))
        require(velocityEcefMetersPerSecond.size == 3 && velocityEcefMetersPerSecond.all(Double::isFinite))
        require(clockBiasMeters.isFinite() && clockDriftMetersPerSecond.isFinite())
        require(accuracyMeters.isFinite() && accuracyMeters > 0.0)
    }
}

/** Deterministic short-horizon provider for tests and frozen replay snapshots. */
class SnapshotSatelliteStateProvider(
    private val snapshots: List<SatelliteStateSnapshot>,
    private val maximumAgeSeconds: Double = 120.0,
) : SatelliteStateProvider {
    init {
        require(maximumAgeSeconds > 0.0)
    }

    override fun stateAt(signal: SatelliteSignalKey, transmitTimeNanos: Long): SatelliteStateResult {
        if (transmitTimeNanos <= 0L) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.INVALID_TIME)
        }
        val candidates = snapshots.filter { snapshot ->
            snapshot.signal.constellation == signal.constellation &&
                snapshot.signal.svid == signal.svid &&
                signalMatches(snapshot.signal, signal)
        }
        val snapshot = candidates.minByOrNull { abs(it.referenceTimeNanos - transmitTimeNanos) }
            ?: return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.UNAVAILABLE_EPHEMERIS)
        if (snapshot.health == SatelliteHealth.UNHEALTHY) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.UNHEALTHY_SATELLITE)
        }
        val deltaSeconds = (transmitTimeNanos - snapshot.referenceTimeNanos) * 1.0e-9
        if (abs(deltaSeconds) > maximumAgeSeconds) {
            return SatelliteStateResult.Unavailable(SatelliteStateFailureReason.STALE_EPHEMERIS)
        }
        return SatelliteStateResult.Available(SatelliteState(
            transmitTimeNanos = transmitTimeNanos,
            positionEcefMeters = DoubleArray(3) {
                snapshot.positionEcefMeters[it] + snapshot.velocityEcefMetersPerSecond[it] * deltaSeconds
            },
            velocityEcefMetersPerSecond = snapshot.velocityEcefMetersPerSecond.copyOf(),
            clockBiasMeters = snapshot.clockBiasMeters + snapshot.clockDriftMetersPerSecond * deltaSeconds,
            clockDriftMetersPerSecond = snapshot.clockDriftMetersPerSecond,
            health = snapshot.health,
            sourceAgeSeconds = abs(deltaSeconds),
            accuracyMeters = snapshot.accuracyMeters,
        ))
    }

    private fun signalMatches(left: SatelliteSignalKey, right: SatelliteSignalKey): Boolean {
        val codeMatches = left.codeType.isBlank() || right.codeType.isBlank() ||
            left.codeType.equals(right.codeType, ignoreCase = true)
        val frequencyMatches = left.carrierFrequencyHz == null || right.carrierFrequencyHz == null ||
            abs(left.carrierFrequencyHz - right.carrierFrequencyHz) <= 2.0e6
        return codeMatches && frequencyMatches
    }
}
