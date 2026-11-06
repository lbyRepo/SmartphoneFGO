package com.example.navgraph.logging

import android.annotation.SuppressLint
import android.content.Context
import android.location.GnssClock
import android.location.GnssMeasurement
import android.location.GnssMeasurementRequest
import android.location.GnssMeasurementsEvent
import android.location.LocationManager
import android.os.Build
import android.os.SystemClock
import com.example.navgraph.live.NavigationEventSink
import com.example.navgraph.live.RawGnssClockSample
import com.example.navgraph.live.RawGnssEpochSample
import com.example.navgraph.live.RawGnssObservationSample
import org.json.JSONObject
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong

class GnssRawCollector(
    context: Context,
    private val writer: NdjsonLogWriter,
    private val eventSink: NavigationEventSink? = null,
) {
    val epochCount = AtomicLong()
    val measurementCount = AtomicLong()
    val lastClockDiscontinuity = AtomicInteger()
    private val manager = context.getSystemService(LocationManager::class.java)
    private val executor = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "navgraph-gnss")
    }
    private val callback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            val callbackElapsed = SystemClock.elapsedRealtimeNanos()
            val eventElapsed = if (event.clock.hasElapsedRealtimeNanos()) {
                event.clock.elapsedRealtimeNanos
            } else {
                callbackElapsed
            }
            writeClock(event.clock, eventElapsed, callbackElapsed)
            event.measurements.forEach { writeMeasurement(it, eventElapsed, callbackElapsed) }
            eventSink?.offer(RawGnssEpochSample(
                elapsedRealtimeNanos = eventElapsed,
                clock = event.clock.toSample(eventElapsed, callbackElapsed),
                observations = event.measurements.map { it.toSample() },
            ))
            epochCount.incrementAndGet()
            measurementCount.addAndGet(event.measurements.size.toLong())
            writer.flush()
        }
    }

    @SuppressLint("MissingPermission")
    fun start() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            val request = GnssMeasurementRequest.Builder()
                .setFullTracking(true)
                .build()
            manager.registerGnssMeasurementsCallback(request, executor, callback)
        } else {
            manager.registerGnssMeasurementsCallback(executor, callback)
        }
    }

    fun stop() {
        manager.unregisterGnssMeasurementsCallback(callback)
        executor.shutdown()
    }

    private fun writeClock(clock: GnssClock, eventElapsed: Long, callbackElapsed: Long) {
        lastClockDiscontinuity.set(clock.hardwareClockDiscontinuityCount)
        val payload = JSONObject()
            .put("time_nanos", clock.timeNanos)
            .put("hardware_clock_discontinuity_count", clock.hardwareClockDiscontinuityCount)
            .put("callback_elapsed_realtime_nanos", callbackElapsed)
        if (clock.hasLeapSecond()) payload.put("leap_second", clock.leapSecond)
        if (clock.hasTimeUncertaintyNanos()) payload.put("time_uncertainty_nanos", clock.timeUncertaintyNanos)
        if (clock.hasFullBiasNanos()) payload.put("full_bias_nanos", clock.fullBiasNanos)
        if (clock.hasBiasNanos()) payload.put("bias_nanos", clock.biasNanos)
        if (clock.hasBiasUncertaintyNanos()) payload.put("bias_uncertainty_nanos", clock.biasUncertaintyNanos)
        if (clock.hasDriftNanosPerSecond()) payload.put("drift_nanos_per_second", clock.driftNanosPerSecond)
        if (clock.hasDriftUncertaintyNanosPerSecond()) {
            payload.put("drift_uncertainty_nanos_per_second", clock.driftUncertaintyNanosPerSecond)
        }
        if (clock.hasElapsedRealtimeNanos()) {
            payload.put("elapsed_realtime_nanos_from_clock", clock.elapsedRealtimeNanos)
            if (clock.hasElapsedRealtimeUncertaintyNanos()) {
                payload.put("elapsed_realtime_uncertainty_nanos", clock.elapsedRealtimeUncertaintyNanos)
            }
        }
        writer.write("gnss_clock", payload, eventElapsed)
    }

    private fun writeMeasurement(measurement: GnssMeasurement, eventElapsed: Long, callbackElapsed: Long) {
        val payload = JSONObject()
            .put("gnss_event_elapsed_realtime_nanos", eventElapsed)
            .put("callback_elapsed_realtime_nanos", callbackElapsed)
            .put("constellation_type", measurement.constellationType)
            .put("svid", measurement.svid)
            .put("state", measurement.state)
            .put("time_offset_nanos", measurement.timeOffsetNanos)
            .put("received_sv_time_nanos", measurement.receivedSvTimeNanos)
            .put("received_sv_time_uncertainty_nanos", measurement.receivedSvTimeUncertaintyNanos)
            .put("cn0_dbhz", measurement.cn0DbHz)
            .put("pseudorange_rate_mps", measurement.pseudorangeRateMetersPerSecond)
            .put("pseudorange_rate_uncertainty_mps", measurement.pseudorangeRateUncertaintyMetersPerSecond)
            .put("accumulated_delta_range_state", measurement.accumulatedDeltaRangeState)
            .put("accumulated_delta_range_m", measurement.accumulatedDeltaRangeMeters)
            .put("accumulated_delta_range_uncertainty_m", measurement.accumulatedDeltaRangeUncertaintyMeters)
            .put("multipath_indicator", measurement.multipathIndicator)
        if (measurement.hasCarrierFrequencyHz()) payload.put("carrier_frequency_hz", measurement.carrierFrequencyHz)
        payload.put("code_type", measurement.codeType)
        if (measurement.hasFullInterSignalBiasNanos()) {
            payload.put("full_inter_signal_bias_nanos", measurement.fullInterSignalBiasNanos)
            payload.put("full_inter_signal_bias_uncertainty_nanos", measurement.fullInterSignalBiasUncertaintyNanos)
        }
        if (measurement.hasSatelliteInterSignalBiasNanos()) {
            payload.put("satellite_inter_signal_bias_nanos", measurement.satelliteInterSignalBiasNanos)
            payload.put("satellite_inter_signal_bias_uncertainty_nanos", measurement.satelliteInterSignalBiasUncertaintyNanos)
        }
        if (measurement.hasBasebandCn0DbHz()) {
            payload.put("baseband_cn0_dbhz", measurement.basebandCn0DbHz)
        }
        if (measurement.hasAutomaticGainControlLevelDb()) {
            payload.put("automatic_gain_control_level_db", measurement.automaticGainControlLevelDb)
        }
        writer.write("gnss_raw", payload, eventElapsed)
    }

    private fun GnssClock.toSample(eventElapsed: Long, callbackElapsed: Long) = RawGnssClockSample(
        elapsedRealtimeNanos = eventElapsed,
        callbackElapsedRealtimeNanos = callbackElapsed,
        timeNanos = timeNanos,
        hardwareClockDiscontinuityCount = hardwareClockDiscontinuityCount,
        fullBiasNanos = if (hasFullBiasNanos()) fullBiasNanos else null,
        biasNanos = if (hasBiasNanos()) biasNanos else null,
        biasUncertaintyNanos = if (hasBiasUncertaintyNanos()) biasUncertaintyNanos else null,
        driftNanosPerSecond = if (hasDriftNanosPerSecond()) driftNanosPerSecond else null,
        driftUncertaintyNanosPerSecond = if (hasDriftUncertaintyNanosPerSecond()) {
            driftUncertaintyNanosPerSecond
        } else null,
    )

    private fun GnssMeasurement.toSample() = RawGnssObservationSample(
        constellation = constellationType,
        svid = svid,
        state = state,
        timeOffsetNanos = timeOffsetNanos,
        receivedSvTimeNanos = receivedSvTimeNanos,
        receivedSvTimeUncertaintyNanos = receivedSvTimeUncertaintyNanos,
        cn0DbHz = cn0DbHz,
        pseudorangeRateMetersPerSecond = pseudorangeRateMetersPerSecond,
        pseudorangeRateUncertaintyMetersPerSecond = pseudorangeRateUncertaintyMetersPerSecond,
        carrierFrequencyHz = if (hasCarrierFrequencyHz()) carrierFrequencyHz.toDouble() else null,
        codeType = codeType,
        fullInterSignalBiasNanos = if (hasFullInterSignalBiasNanos()) fullInterSignalBiasNanos else null,
        satelliteInterSignalBiasNanos = if (hasSatelliteInterSignalBiasNanos()) {
            satelliteInterSignalBiasNanos
        } else null,
    )
}
