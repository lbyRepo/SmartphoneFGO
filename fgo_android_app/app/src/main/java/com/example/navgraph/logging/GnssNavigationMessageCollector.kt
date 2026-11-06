package com.example.navgraph.logging

import android.annotation.SuppressLint
import android.content.Context
import android.location.GnssNavigationMessage
import android.location.LocationManager
import android.os.SystemClock
import android.util.Base64
import com.example.navgraph.live.NavigationEventSink
import com.example.navgraph.live.RawGnssNavigationMessageSample
import org.json.JSONObject
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicLong

class GnssNavigationMessageCollector(
    context: Context,
    private val writer: NdjsonLogWriter,
    private val eventSink: NavigationEventSink? = null,
) {
    val messageCount = AtomicLong()
    private val manager = context.getSystemService(LocationManager::class.java)
    private val executor = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "navgraph-navigation-message")
    }
    private val callback = object : GnssNavigationMessage.Callback() {
        override fun onGnssNavigationMessageReceived(event: GnssNavigationMessage) {
            val callbackElapsed = SystemClock.elapsedRealtimeNanos()
            val data = event.data.copyOf()
            writer.write(
                "gnss_navigation_message",
                JSONObject()
                    .put("callback_elapsed_realtime_nanos", callbackElapsed)
                    .put("svid", event.svid)
                    .put("message_type", event.type)
                    .put("message_id", event.messageId)
                    .put("submessage_id", event.submessageId)
                    .put("status", event.status)
                    .put("data_base64", Base64.encodeToString(data, Base64.NO_WRAP)),
                callbackElapsed,
            )
            eventSink?.offer(RawGnssNavigationMessageSample(
                elapsedRealtimeNanos = callbackElapsed,
                svid = event.svid,
                messageType = event.type,
                messageId = event.messageId,
                submessageId = event.submessageId,
                status = event.status,
                data = data,
            ))
            messageCount.incrementAndGet()
        }
    }

    @SuppressLint("MissingPermission")
    fun start(): Boolean = manager.registerGnssNavigationMessageCallback(executor, callback)

    fun stop() {
        manager.unregisterGnssNavigationMessageCallback(callback)
        executor.shutdown()
    }
}
