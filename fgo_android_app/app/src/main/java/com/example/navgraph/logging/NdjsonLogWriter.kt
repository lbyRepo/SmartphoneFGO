package com.example.navgraph.logging

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
    val file: File,
    val sessionId: String,
    private val stream: BufferedWriter,
) : Closeable {
    private var closed = false

    @Synchronized
    fun write(recordType: String, payload: JSONObject, eventElapsedNanos: Long) {
        if (closed) return
        payload.put("schema_version", 1)
        payload.put("record_type", recordType)
        payload.put("session_id", sessionId)
        payload.put("elapsed_realtime_nanos", eventElapsedNanos)
        payload.put("wall_time_millis", System.currentTimeMillis())
        stream.write(payload.toString())
        stream.newLine()
    }

    @Synchronized
    fun flush() {
        if (!closed) stream.flush()
    }

    @Synchronized
    fun close(reason: String) {
        if (closed) return
        write("session_end", JSONObject().put("reason", reason), SystemClock.elapsedRealtimeNanos())
        stream.flush()
        closed = true
        stream.close()
    }

    override fun close() = close("closed")

    companion object {
        private val timestampFormatter = DateTimeFormatter
            .ofPattern("yyyyMMdd_HHmmss")
            .withZone(ZoneOffset.UTC)

        fun create(context: Context, requestedImuPeriodUs: Int): NdjsonLogWriter {
            val documents = context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS)
                ?: context.filesDir
            val logDirectory = File(documents, "logs").apply { mkdirs() }
            val suffix = UUID.randomUUID().toString().take(8)
            val sessionId = "${timestampFormatter.format(Instant.now())}_$suffix"
            val file = File(logDirectory, "navgraph_$sessionId.ndjson")
            val writer = NdjsonLogWriter(
                file,
                sessionId,
                BufferedWriter(OutputStreamWriter(FileOutputStream(file), Charsets.UTF_8), 64 * 1024),
            )
            writer.write(
                "session_start",
                JSONObject()
                    .put("app_version", "0.12.0")
                    .put("android_sdk", Build.VERSION.SDK_INT)
                    .put("device_manufacturer", Build.MANUFACTURER)
                    .put("device_model", Build.MODEL)
                    .put("device", Build.DEVICE)
                    .put("hardware", Build.HARDWARE)
                    .put("fingerprint", Build.FINGERPRINT)
                    .put("requested_imu_period_us", requestedImuPeriodUs),
                SystemClock.elapsedRealtimeNanos(),
            )
            return writer
        }
    }
}
