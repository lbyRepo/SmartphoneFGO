package com.example.s23logger.gnss

import android.content.Context
import android.location.GnssClock
import android.location.GnssMeasurement
import android.location.GnssMeasurementRequest
import android.location.GnssMeasurementsEvent
import android.location.LocationManager
import android.os.Build
import android.os.SystemClock
import com.example.s23logger.logging.NdjsonLogWriter
import org.json.JSONObject
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

class GnssRawCollector(
    context: Context,
    private val writer: NdjsonLogWriter,
) {
    private val locationManager = context.getSystemService(LocationManager::class.java)
    private val executor: ExecutorService = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "s23-gnss-logger")
    }

    private val callback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(eventArgs: GnssMeasurementsEvent) {
            val receiptElapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos()
            writeClock(eventArgs.clock, receiptElapsedRealtimeNanos)
            eventArgs.measurements.forEach { measurement ->
                writeMeasurement(measurement, receiptElapsedRealtimeNanos)
            }
            writer.flush()
        }
    }

    fun start() {
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                val request = GnssMeasurementRequest.Builder()
                    .setFullTracking(true)
                    .build()
                locationManager.registerGnssMeasurementsCallback(request, executor, callback)
            } else {
                locationManager.registerGnssMeasurementsCallback(executor, callback)
            }
        } catch (_: SecurityException) {
            // Permission handling lives in MainActivity; keep the collector side-effect free on failure.
        }
    }

    fun stop() {
        locationManager.unregisterGnssMeasurementsCallback(callback)
        executor.shutdownNow()
    }

    private fun writeClock(clock: GnssClock, receiptElapsedRealtimeNanos: Long) {
        val payload = JSONObject()
            .put("time_nanos", clock.timeNanos)
            .put("hardware_clock_discontinuity_count", clock.hardwareClockDiscontinuityCount)

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
        }
        if (clock.hasElapsedRealtimeUncertaintyNanos()) {
            payload.put("elapsed_realtime_uncertainty_nanos", clock.elapsedRealtimeUncertaintyNanos)
        }

        writer.write("gnss_clock", payload, receiptElapsedRealtimeNanos)
    }

    private fun writeMeasurement(measurement: GnssMeasurement, gnssEventElapsedRealtimeNanos: Long) {
        val payload = JSONObject()
            .put("gnss_event_elapsed_realtime_nanos", gnssEventElapsedRealtimeNanos)
            .put("constellation_type", measurement.constellationType)
            .put("svid", measurement.svid)
            .put("state", measurement.state)
            .put("received_sv_time_nanos", measurement.receivedSvTimeNanos)
            .put("received_sv_time_uncertainty_nanos", measurement.receivedSvTimeUncertaintyNanos)
            .put("cn0_dbhz", measurement.cn0DbHz)
            .put("pseudorange_rate_mps", measurement.pseudorangeRateMetersPerSecond)
            .put("pseudorange_rate_uncertainty_mps", measurement.pseudorangeRateUncertaintyMetersPerSecond)
            .put("accumulated_delta_range_state", measurement.accumulatedDeltaRangeState)
            .put("accumulated_delta_range_m", measurement.accumulatedDeltaRangeMeters)
            .put("accumulated_delta_range_uncertainty_m", measurement.accumulatedDeltaRangeUncertaintyMeters)
            .put("multipath_indicator", measurement.multipathIndicator)

        if (measurement.hasCarrierFrequencyHz()) payload.put("carrier_frequency_hz", measurement.carrierFrequencyHz.toDouble())
        if (measurement.hasCodeType()) payload.put("code_type", measurement.codeType)
        if (measurement.hasBasebandCn0DbHz()) payload.put("baseband_cn0_dbhz", measurement.basebandCn0DbHz)
        if (measurement.hasAutomaticGainControlLevelDb()) {
            payload.put("automatic_gain_control_db", measurement.automaticGainControlLevelDb)
        }
        if (measurement.hasSatelliteInterSignalBiasNanos()) {
            payload.put("satellite_inter_signal_bias_nanos", measurement.satelliteInterSignalBiasNanos)
        }
        if (measurement.hasSatelliteInterSignalBiasUncertaintyNanos()) {
            payload.put(
                "satellite_inter_signal_bias_uncertainty_nanos",
                measurement.satelliteInterSignalBiasUncertaintyNanos,
            )
        }
        if (measurement.hasFullInterSignalBiasNanos()) {
            payload.put("full_inter_signal_bias_nanos", measurement.fullInterSignalBiasNanos)
        }
        if (measurement.hasFullInterSignalBiasUncertaintyNanos()) {
            payload.put("full_inter_signal_bias_uncertainty_nanos", measurement.fullInterSignalBiasUncertaintyNanos)
        }

        writer.write("gnss_raw", payload, gnssEventElapsedRealtimeNanos)
    }
}
