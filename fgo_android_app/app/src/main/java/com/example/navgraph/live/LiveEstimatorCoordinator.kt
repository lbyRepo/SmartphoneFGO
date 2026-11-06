package com.example.navgraph.live

import com.example.navgraph.estimator.GnssEpoch
import com.example.navgraph.preprocessing.ConservativeCorrectionProvider
import com.example.navgraph.preprocessing.EphemerisHealthSource
import com.example.navgraph.preprocessing.NavigationMessageIngestor
import com.example.navgraph.preprocessing.RawGnssPreprocessor
import com.example.navgraph.preprocessing.SatelliteStateProvider
import com.example.navgraph.preprocessing.UnavailableSatelliteStateProvider

interface LiveEstimatorBackend {
    fun reset(clockContinuityId: Int)
    fun pushImu(sample: RawImuSample)
    fun pushGnssEpoch(epoch: GnssEpoch)
    fun updateFusedReference(sample: FusedLocationSample)
    fun close() = Unit
}

object NoOpLiveEstimatorBackend : LiveEstimatorBackend {
    override fun reset(clockContinuityId: Int) = Unit
    override fun pushImu(sample: RawImuSample) = Unit
    override fun pushGnssEpoch(epoch: GnssEpoch) = Unit
    override fun updateFusedReference(sample: FusedLocationSample) = Unit
}

interface LiveEstimatorBackendHealthSource {
    fun backendHealth(): LiveBackendHealth
}

data class LiveEstimatorHealth(
    val bus: NavigationBusHealth,
    val imuEvents: Long,
    val gnssEpochs: Long,
    val fusedFixes: Long,
    val cleanGnssEpochs: Long,
    val acceptedGnssObservations: Long,
    val rejectedGnssObservations: Long,
    val unavailableEphemerisCount: Long,
    val clockResetCount: Long,
    val clockContinuityId: Int?,
    val latestGnssState: String,
    val navigationMessages: Long,
    val completeEphemerides: Int,
    val rejectedNavigationMessages: Long,
    val latestEphemerisRejection: String,
    val backend: LiveBackendHealth,
)

class LiveEstimatorCoordinator(
    satelliteStateProvider: SatelliteStateProvider = UnavailableSatelliteStateProvider,
    private val backend: LiveEstimatorBackend = NoOpLiveEstimatorBackend,
    queueCapacity: Int = 4_096,
) : NavigationEventSink, AutoCloseable {
    private val preprocessor = RawGnssPreprocessor(
        satelliteStateProvider,
        ConservativeCorrectionProvider(),
    )
    private val healthLock = Any()
    private val rejectionTotals = linkedMapOf<String, Long>()
    private var imuEvents = 0L
    private var gnssEpochs = 0L
    private var fusedFixes = 0L
    private var cleanGnssEpochs = 0L
    private var acceptedGnssObservations = 0L
    private var clockResetCount = 0L
    private var clockContinuityId: Int? = null
    private var latestGnssState = "waiting_for_raw_gnss"
    private var navigationMessages = 0L
    private val navigationMessageIngestor = satelliteStateProvider as? NavigationMessageIngestor
    private val ephemerisHealthSource = satelliteStateProvider as? EphemerisHealthSource
    private val bus = BoundedNavigationEventBus(queueCapacity, ::consume)

    init {
        bus.start()
    }

    override fun offer(event: NavigationEvent): Boolean = bus.offer(event)

    fun health(): LiveEstimatorHealth = synchronized(healthLock) {
        val ephemeris = ephemerisHealthSource?.ephemerisHealth()
        LiveEstimatorHealth(
            bus = bus.health(),
            imuEvents = imuEvents,
            gnssEpochs = gnssEpochs,
            fusedFixes = fusedFixes,
            cleanGnssEpochs = cleanGnssEpochs,
            acceptedGnssObservations = acceptedGnssObservations,
            rejectedGnssObservations = rejectionTotals.values.sum(),
            unavailableEphemerisCount = rejectionTotals["unavailable_ephemeris"] ?: 0L,
            clockResetCount = clockResetCount,
            clockContinuityId = clockContinuityId,
            latestGnssState = latestGnssState,
            navigationMessages = navigationMessages,
            completeEphemerides = ephemeris?.completeEphemerides ?: 0,
            rejectedNavigationMessages = ephemeris?.rejectedMessages ?: 0L,
            latestEphemerisRejection = ephemeris?.lastRejectionReason ?: "no_provider",
            backend = (backend as? LiveEstimatorBackendHealthSource)?.backendHealth()
                ?: LiveBackendHealth(),
        )
    }

    override fun close() {
        bus.close()
        backend.close()
    }

    private fun consume(event: NavigationEvent) {
        when (event) {
            is RawImuSample -> {
                backend.pushImu(event)
                synchronized(healthLock) { imuEvents++ }
            }
            is FusedLocationSample -> {
                preprocessor.updateApproximateReceiver(
                    event.latitudeDegrees,
                    event.longitudeDegrees,
                    event.altitudeMeters ?: 0.0,
                )
                backend.updateFusedReference(event)
                synchronized(healthLock) { fusedFixes++ }
            }
            is RawGnssEpochSample -> consumeGnss(event)
            is RawGnssNavigationMessageSample -> {
                navigationMessageIngestor?.ingest(event)
                synchronized(healthLock) { navigationMessages++ }
            }
            is RawGnssClockSample -> Unit
        }
    }

    private fun consumeGnss(event: RawGnssEpochSample) {
        val continuity = event.clock.hardwareClockDiscontinuityCount
        val changed = synchronized(healthLock) {
            val previous = clockContinuityId
            clockContinuityId = continuity
            previous != null && previous != continuity
        }
        if (changed) {
            backend.reset(continuity)
            synchronized(healthLock) { clockResetCount++ }
        }
        val result = preprocessor.consume(event)
        result.epoch?.let(backend::pushGnssEpoch)
        synchronized(healthLock) {
            gnssEpochs++
            acceptedGnssObservations += result.acceptedObservationCount
            result.rejectionCounts.forEach { (reason, count) ->
                rejectionTotals[reason] = (rejectionTotals[reason] ?: 0L) + count
            }
            if (result.epoch != null) cleanGnssEpochs++
            latestGnssState = when {
                result.epoch != null -> "clean_epoch_ready"
                result.rejectionCounts.containsKey("unavailable_ephemeris") -> "waiting_for_ephemeris"
                result.rejectionCounts.containsKey("missing_clock_bias") -> "invalid_receiver_clock"
                else -> result.rejectionCounts.maxByOrNull { it.value }?.key ?: "no_usable_observations"
            }
        }
    }
}
