package com.example.s23logger.logging

import android.content.Context
import android.os.Build
import android.os.Environment
import android.os.SystemClock
import org.json.JSONObject
import java.io.BufferedWriter
import java.io.Closeable
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStreamWriter
import java.time.Instant
import java.time.ZoneOffset
import java.time.format.DateTimeFormatter
import java.util.UUID

class NdjsonLogWriter private constructor(
    private val context: Context,
    val file: File,
    private val sessionId: String,
    private val writer: BufferedWriter,
) : Closeable {
    @Synchronized
    fun write(recordType: String, payload: JSONObject, elapsedRealtimeNanos: Long = SystemClock.elapsedRealtimeNanos()) {
        payload.put("schema_version", SCHEMA_VERSION)
        payload.put("record_type", recordType)
        payload.put("session_id", sessionId)
        payload.put("elapsed_realtime_nanos", elapsedRealtimeNanos)
        payload.put("wall_time_millis", System.currentTimeMillis())
        writer.write(payload.toString())
        writer.newLine()
    }

    @Synchronized
    fun flush() {
        writer.flush()
    }

    @Synchronized
    fun close(reason: String) {
        write("session_stop", JSONObject().put("reason", reason))
        writer.flush()
        writer.close()
    }

    override fun close() {
        close("closed")
    }

    private fun writeSessionStart(requestedImuPeriodUs: Int) {
        val packageInfo = context.packageManager.getPackageInfo(context.packageName, 0)
        write(
            "session_start",
            JSONObject()
                .put("app_version", packageInfo.versionName ?: "unknown")
                .put("android_sdk", Build.VERSION.SDK_INT)
                .put("device_manufacturer", Build.MANUFACTURER)
                .put("device_model", Build.MODEL)
                .put("device", Build.DEVICE)
                .put("hardware", Build.HARDWARE)
                .put("fingerprint", Build.FINGERPRINT)
                .put("requested_imu_period_us", requestedImuPeriodUs),
        )
    }

    companion object {
        const val SCHEMA_VERSION = 1
        private val SESSION_FORMATTER: DateTimeFormatter =
            DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss").withZone(ZoneOffset.UTC)

        fun create(context: Context, requestedImuPeriodUs: Int): NdjsonLogWriter {
            val sessionId = "${SESSION_FORMATTER.format(Instant.now())}_${UUID.randomUUID().toString().take(8)}"
            val logDir = File(
                context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS),
                "logs",
            )
            check(logDir.mkdirs() || logDir.isDirectory) { "Unable to create log directory: $logDir" }
            val file = File(logDir, "s23_gnss_logger_$sessionId.ndjson")
            val bufferedWriter = BufferedWriter(OutputStreamWriter(FileOutputStream(file), Charsets.UTF_8))
            return NdjsonLogWriter(context.applicationContext, file, sessionId, bufferedWriter).also {
                it.writeSessionStart(requestedImuPeriodUs)
            }
        }
    }
}
