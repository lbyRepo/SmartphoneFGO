package com.example.s23logger.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import com.example.s23logger.logging.NdjsonLogWriter
import org.json.JSONArray
import org.json.JSONObject

class ImuCollector(
    context: Context,
    private val writer: NdjsonLogWriter,
    private val requestedPeriodUs: Int,
) : SensorEventListener {
    private val sensorManager = context.getSystemService(SensorManager::class.java)
    private val handlerThread = HandlerThread("s23-imu-logger")
    private val sensors = listOfNotNull(
        sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
        sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
    )

    fun start() {
        handlerThread.start()
        val handler = Handler(handlerThread.looper)
        sensors.forEach { sensor ->
            writeSensorMeta(sensor)
            sensorManager.registerListener(this, sensor, requestedPeriodUs, 0, handler)
        }
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        handlerThread.quitSafely()
    }

    override fun onSensorChanged(event: SensorEvent) {
        val values = JSONArray()
        for (index in 0 until minOf(3, event.values.size)) {
            values.put(event.values[index].toDouble())
        }
        while (values.length() < 3) {
            values.put(0.0)
        }

        writer.write(
            "imu",
            JSONObject()
                .put("sensor_type", sensorTypeName(event.sensor.type))
                .put("sensor_name", event.sensor.name)
                .put("sensor_timestamp_nanos", event.timestamp)
                .put("accuracy", event.accuracy)
                .put("values", values),
        )
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun writeSensorMeta(sensor: Sensor) {
        writer.write(
            "sensor_meta",
            JSONObject()
                .put("sensor_type", sensorTypeName(sensor.type))
                .put("sensor_name", sensor.name)
                .put("vendor", sensor.vendor)
                .put("version", sensor.version)
                .put("resolution", sensor.resolution.toDouble())
                .put("maximum_range", sensor.maximumRange.toDouble())
                .put("power_ma", sensor.power.toDouble())
                .put("min_delay_us", sensor.minDelay)
                .put("max_delay_us", sensor.maxDelay)
                .put("fifo_reserved_event_count", sensor.fifoReservedEventCount)
                .put("fifo_max_event_count", sensor.fifoMaxEventCount)
                .put("reporting_mode", sensor.reportingMode)
                .put("wake_up", sensor.isWakeUpSensor),
        )
    }

    private fun sensorTypeName(type: Int): String = when (type) {
        Sensor.TYPE_ACCELEROMETER -> "ACCELEROMETER"
        Sensor.TYPE_GYROSCOPE -> "GYROSCOPE"
        else -> "SENSOR_$type"
    }
}
