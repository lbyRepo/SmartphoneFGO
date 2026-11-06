package com.example.s23logger.location

import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.HandlerThread
import android.os.SystemClock
import com.example.s23logger.logging.NdjsonLogWriter
import org.json.JSONObject

class FusedLocationCollector(
    context: Context,
    private val writer: NdjsonLogWriter,
) {
    private val locationManager = context.getSystemService(LocationManager::class.java)
    private val handlerThread = HandlerThread("s23-fused-location-logger")

    private val listener = object : LocationListener {
        override fun onLocationChanged(location: Location) {
            writeLocation(location)
        }

        @Deprecated("Deprecated by Android framework")
        override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) = Unit
    }

    @SuppressLint("MissingPermission")
    fun start() {
        handlerThread.start()
        try {
            locationManager.getLastKnownLocation(LocationManager.FUSED_PROVIDER)?.let { writeLocation(it) }
            locationManager.requestLocationUpdates(
                LocationManager.FUSED_PROVIDER,
                LOCATION_INTERVAL_MILLIS,
                0f,
                listener,
                handlerThread.looper,
            )
        } catch (_: RuntimeException) {
            // Some devices may not expose the fused provider to platform clients.
        }
    }

    fun stop() {
        locationManager.removeUpdates(listener)
        handlerThread.quitSafely()
    }

    private fun writeLocation(location: Location) {
        val payload = JSONObject()
            .put("provider", location.provider ?: LocationManager.FUSED_PROVIDER)
            .put("location_elapsed_realtime_nanos", location.elapsedRealtimeNanos)
            .put("latitude_deg", location.latitude)
            .put("longitude_deg", location.longitude)
            .put("time_millis", location.time)

        if (location.hasAltitude()) payload.put("altitude_m", location.altitude)
        if (location.hasSpeed()) payload.put("speed_mps", location.speed.toDouble())
        if (location.hasBearing()) payload.put("bearing_deg", location.bearing.toDouble())
        if (location.hasAccuracy()) payload.put("horizontal_accuracy_m", location.accuracy.toDouble())
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && location.hasVerticalAccuracy()) {
            payload.put("vertical_accuracy_m", location.verticalAccuracyMeters.toDouble())
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && location.hasSpeedAccuracy()) {
            payload.put("speed_accuracy_mps", location.speedAccuracyMetersPerSecond.toDouble())
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && location.hasBearingAccuracy()) {
            payload.put("bearing_accuracy_deg", location.bearingAccuracyDegrees.toDouble())
        }
        val isMock = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            location.isMock
        } else {
            @Suppress("DEPRECATION")
            location.isFromMockProvider
        }
        payload.put("mock", isMock)

        writer.write("fused_location", payload, SystemClock.elapsedRealtimeNanos())
    }

    companion object {
        private const val LOCATION_INTERVAL_MILLIS = 1_000L
    }
}
