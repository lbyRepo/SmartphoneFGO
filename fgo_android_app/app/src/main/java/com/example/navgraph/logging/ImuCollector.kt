package com.example.navgraph.logging

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import com.example.navgraph.live.ImuSensorType
import com.example.navgraph.live.NavigationEventSink
import com.example.navgraph.live.RawImuSample
import org.json.JSONArray
import org.json.JSONObject
import java.util.concurrent.atomic.AtomicLong

class ImuCollector(
    context: Context,
    private val writer: NdjsonLogWriter,
    private val samplingPeriodUs: Int,
    private val eventSink: NavigationEventSink? = null,
) : SensorEventListener {
    val sampleCount = AtomicLong()
    private val sensorManager = context.getSystemService(SensorManager::class.java)
    private val thread = HandlerThread("navgraph-imu")
    private var handler: Handler? = null
    private val sensors = listOfNotNull(
        sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
        sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
    )

    fun start() {
        thread.start()
        handler = Handler(thread.looper)
        sensors.forEach { sensor ->
            writeMetadata(sensor)
            sensorManager.registerListener(this, sensor, samplingPeriodUs, 0, handler)
        }
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        thread.quitSafely()
    }

    override fun onSensorChanged(event: SensorEvent) {
        val type = when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> "ACCELEROMETER"
            Sensor.TYPE_GYROSCOPE -> "GYROSCOPE"
            else -> return
        }
        val values = JSONArray()
        event.values.forEach(values::put)
        val callbackElapsed = SystemClock.elapsedRealtimeNanos()
        writer.write(
            "imu",
            JSONObject()
                .put("sensor_type", type)
                .put("sensor_name", event.sensor.name)
                .put("sensor_timestamp_nanos", event.timestamp)
                .put("callback_elapsed_realtime_nanos", callbackElapsed)
                .put("accuracy", event.accuracy)
                .put("values", values),
            event.timestamp,
        )
        eventSink?.offer(RawImuSample(
            elapsedRealtimeNanos = event.timestamp,
            callbackElapsedRealtimeNanos = callbackElapsed,
            sensorType = if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
                ImuSensorType.ACCELEROMETER
            } else {
                ImuSensorType.GYROSCOPE
            },
            values = DoubleArray(3) { event.values[it].toDouble() },
            accuracy = event.accuracy,
        ))
        sampleCount.incrementAndGet()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun writeMetadata(sensor: Sensor) {
        writer.write(
            "sensor_meta",
            JSONObject()
                .put("sensor_type", if (sensor.type == Sensor.TYPE_ACCELEROMETER) "ACCELEROMETER" else "GYROSCOPE")
                .put("sensor_name", sensor.name)
                .put("vendor", sensor.vendor)
                .put("version", sensor.version)
                .put("resolution", sensor.resolution)
                .put("maximum_range", sensor.maximumRange)
                .put("power_ma", sensor.power)
                .put("min_delay_us", sensor.minDelay)
                .put("max_delay_us", sensor.maxDelay)
                .put("fifo_reserved_event_count", sensor.fifoReservedEventCount)
                .put("fifo_max_event_count", sensor.fifoMaxEventCount)
                .put("reporting_mode", sensor.reportingMode)
                .put("wake_up", sensor.isWakeUpSensor),
            SystemClock.elapsedRealtimeNanos(),
        )
    }
}
