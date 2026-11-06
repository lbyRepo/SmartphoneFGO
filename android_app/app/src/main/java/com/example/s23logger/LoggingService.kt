package com.example.s23logger

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import android.os.SystemClock
import com.example.s23logger.gnss.GnssRawCollector
import com.example.s23logger.location.FusedLocationCollector
import com.example.s23logger.logging.NdjsonLogWriter
import com.example.s23logger.sensors.ImuCollector
import java.io.File

class LoggingService : Service() {
    private var writer: NdjsonLogWriter? = null
    private var imuCollector: ImuCollector? = null
    private var gnssCollector: GnssRawCollector? = null
    private var fusedLocationCollector: FusedLocationCollector? = null

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_START -> startLogging()
            ACTION_STOP -> {
                stopLogging("user_stop")
                stopSelf(startId)
            }
        }
        return START_STICKY
    }

    override fun onDestroy() {
        stopLogging("service_destroyed")
        super.onDestroy()
    }

    private fun startLogging() {
        if (writer != null) {
            sendStatus("Already logging", writer?.file)
            return
        }
        createNotificationChannel()
        startForegroundCompat(buildNotification(null))

        val logWriter = NdjsonLogWriter.create(this, REQUESTED_IMU_PERIOD_US)
        writer = logWriter
        startForegroundCompat(buildNotification(logWriter.file))

        imuCollector = ImuCollector(this, logWriter, REQUESTED_IMU_PERIOD_US).also { it.start() }
        gnssCollector = GnssRawCollector(this, logWriter).also { it.start() }
        fusedLocationCollector = FusedLocationCollector(this, logWriter).also { it.start() }

        sendStatus("Logging started", logWriter.file)
    }

    private fun stopLogging(reason: String) {
        val logWriter = writer ?: return
        fusedLocationCollector?.stop()
        gnssCollector?.stop()
        imuCollector?.stop()
        fusedLocationCollector = null
        gnssCollector = null
        imuCollector = null
        writer = null
        logWriter.close(reason)
        sendStatus("Logging stopped: $reason", logWriter.file)
        if (Build.VERSION.SDK_INT >= 24) {
            stopForeground(STOP_FOREGROUND_REMOVE)
        } else {
            @Suppress("DEPRECATION")
            stopForeground(true)
        }
    }

    private fun startForegroundCompat(notification: Notification) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            startForeground(
                NOTIFICATION_ID,
                notification,
                ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION,
            )
        } else {
            startForeground(NOTIFICATION_ID, notification)
        }
    }

    private fun buildNotification(file: File?): Notification {
        val stopIntent = Intent(this, LoggingService::class.java).setAction(ACTION_STOP)
        val stopPendingIntent = PendingIntent.getService(
            this,
            0,
            stopIntent,
            PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT,
        )
        val activityIntent = PendingIntent.getActivity(
            this,
            1,
            Intent(this, MainActivity::class.java),
            PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT,
        )
        val text = file?.absolutePath ?: "Preparing GNSS and IMU log"
        return Notification.Builder(this, CHANNEL_ID)
            .setContentTitle(getString(R.string.app_name))
            .setContentText(text)
            .setSmallIcon(R.drawable.ic_stat_logger)
            .setOngoing(true)
            .setContentIntent(activityIntent)
            .addAction(R.drawable.ic_stat_logger, "Stop", stopPendingIntent)
            .build()
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT < 26) return
        val manager = getSystemService(NotificationManager::class.java)
        val channel = NotificationChannel(
            CHANNEL_ID,
            getString(R.string.logging_channel_name),
            NotificationManager.IMPORTANCE_LOW,
        )
        manager.createNotificationChannel(channel)
    }

    private fun sendStatus(message: String, file: File?) {
        sendBroadcast(Intent(ACTION_STATUS).apply {
            setPackage(packageName)
            putExtra(EXTRA_STATUS_MESSAGE, message)
            putExtra(EXTRA_LOG_PATH, file?.absolutePath.orEmpty())
            putExtra(EXTRA_ELAPSED_REALTIME_NANOS, SystemClock.elapsedRealtimeNanos())
        })
    }

    companion object {
        const val ACTION_START = "com.example.s23logger.action.START"
        const val ACTION_STOP = "com.example.s23logger.action.STOP"
        const val ACTION_STATUS = "com.example.s23logger.action.STATUS"
        const val EXTRA_STATUS_MESSAGE = "status_message"
        const val EXTRA_LOG_PATH = "log_path"
        const val EXTRA_ELAPSED_REALTIME_NANOS = "elapsed_realtime_nanos"

        private const val CHANNEL_ID = "gnss_logging"
        private const val NOTIFICATION_ID = 2201
        private const val REQUESTED_IMU_PERIOD_US = 10_000
    }
}
