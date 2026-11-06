package com.example.navgraph.logging

import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.os.HandlerThread
import android.os.SystemClock
import com.example.navgraph.live.FusedLocationSample
import com.example.navgraph.live.NavigationEventSink
import org.json.JSONObject
import java.util.concurrent.atomic.AtomicLong

class FusedLocationCollector(
    context: Context,
    private val writer: NdjsonLogWriter,
    private val onLocation: (Location) -> Unit,
    private val eventSink: NavigationEventSink? = null,
) : LocationListener {
    val locationCount = AtomicLong()
    private val manager = context.getSystemService(LocationManager::class.java)
    private val thread = HandlerThread("navgraph-location")
    private val registeredProviders = mutableListOf<String>()

    @SuppressLint("MissingPermission")
    fun start() {
        thread.start()
        listOf("fused", LocationManager.GPS_PROVIDER, LocationManager.NETWORK_PROVIDER)
            .distinct()
            .filter { it in manager.allProviders && manager.isProviderEnabled(it) }
            .forEach { provider ->
                manager.requestLocationUpdates(provider, 1_000L, 0f, this, thread.looper)
                registeredProviders += provider
            }
    }

    fun stop() {
        manager.removeUpdates(this)
        registeredProviders.clear()
        thread.quitSafely()
    }

    override fun onLocationChanged(location: Location) {
        val callbackElapsed = SystemClock.elapsedRealtimeNanos()
        val payload = JSONObject()
            .put("provider", location.provider)
            .put("latitude_deg", location.latitude)
            .put("longitude_deg", location.longitude)
            .put("location_time_millis", location.time)
            .put("location_elapsed_realtime_nanos", location.elapsedRealtimeNanos)
            .put("callback_elapsed_realtime_nanos", callbackElapsed)
            .put("accuracy_m", location.accuracy)
        if (location.hasAltitude()) payload.put("altitude_m", location.altitude)
        if (location.hasVerticalAccuracy()) payload.put("vertical_accuracy_m", location.verticalAccuracyMeters)
        if (location.hasSpeed()) payload.put("speed_mps", location.speed)
        if (location.hasSpeedAccuracy()) payload.put("speed_accuracy_mps", location.speedAccuracyMetersPerSecond)
        if (location.hasBearing()) payload.put("bearing_deg", location.bearing)
        if (location.hasBearingAccuracy()) payload.put("bearing_accuracy_deg", location.bearingAccuracyDegrees)
        writer.write("fused_location", payload, location.elapsedRealtimeNanos)
        eventSink?.offer(FusedLocationSample(
            elapsedRealtimeNanos = location.elapsedRealtimeNanos,
            callbackElapsedRealtimeNanos = callbackElapsed,
            provider = location.provider.orEmpty(),
            latitudeDegrees = location.latitude,
            longitudeDegrees = location.longitude,
            altitudeMeters = if (location.hasAltitude()) location.altitude else null,
            horizontalAccuracyMeters = location.accuracy.toDouble(),
        ))
        locationCount.incrementAndGet()
        onLocation(location)
    }

    @Deprecated("Deprecated in Android")
    override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) = Unit

    override fun onProviderEnabled(provider: String) = Unit
    override fun onProviderDisabled(provider: String) = Unit
}
