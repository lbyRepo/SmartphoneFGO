package com.example.navgraph.logging

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Intent
import android.content.pm.ServiceInfo
import android.location.Location
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.os.SystemClock
import com.example.navgraph.MainActivity
import com.example.navgraph.estimator.NativeGnssWls
import com.example.navgraph.estimator.NativeGnssInsEkf
import com.example.navgraph.estimator.NativeFixedLagSmoother
import com.example.navgraph.estimator.NativeInsMechanization
import com.example.navgraph.live.KotlinLiveEstimatorBackend
import com.example.navgraph.live.LiveEstimatorCoordinator
import com.example.navgraph.preprocessing.GpsL1CaEphemerisProvider

class LoggingService : Service() {
    private var writer: NdjsonLogWriter? = null
    private var imuCollector: ImuCollector? = null
    private var gnssCollector: GnssRawCollector? = null
    private var navigationMessageCollector: GnssNavigationMessageCollector? = null
    private var fusedCollector: FusedLocationCollector? = null
    private var liveCoordinator: LiveEstimatorCoordinator? = null
    private val handler = Handler(Looper.getMainLooper())
    private var lastLocation: Location? = null
    private var startedElapsedNanos = 0L

    private val statusTask = object : Runnable {
        override fun run() {
            broadcastStatus(logging = writer != null)
            if (writer != null) handler.postDelayed(this, 1_000L)
        }
    }

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_STOP -> stopLogging("user_stop")
            ACTION_QUERY -> broadcastStatus(logging = writer != null)
            else -> startLogging()
        }
        return START_NOT_STICKY
    }

    override fun onDestroy() {
        stopCollectors("service_destroyed")
        super.onDestroy()
    }

    private fun startLogging() {
        if (writer != null) {
            broadcastStatus(logging = true)
            return
        }
        val foregroundNotification = buildNotification("Preparing session")
        startForeground(NOTIFICATION_ID, foregroundNotification, ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION)
        val logWriter = NdjsonLogWriter.create(this, IMU_PERIOD_US)
        writer = logWriter
        startedElapsedNanos = SystemClock.elapsedRealtimeNanos()
        getSharedPreferences(PREFERENCES, MODE_PRIVATE).edit()
            .putBoolean(KEY_LOGGING, true)
            .putString(KEY_LOG_NAME, logWriter.file.name)
            .apply()
        val ephemerisProvider = GpsL1CaEphemerisProvider()
        val coordinator = LiveEstimatorCoordinator(
            satelliteStateProvider = ephemerisProvider,
            backend = KotlinLiveEstimatorBackend(),
        )
        liveCoordinator = coordinator
        imuCollector = ImuCollector(this, logWriter, IMU_PERIOD_US, coordinator).also { it.start() }
        gnssCollector = GnssRawCollector(this, logWriter, coordinator).also { it.start() }
        navigationMessageCollector = GnssNavigationMessageCollector(
            this,
            logWriter,
            coordinator,
        ).also { it.start() }
        fusedCollector = FusedLocationCollector(
            context = this,
            writer = logWriter,
            onLocation = { location -> lastLocation = location },
            eventSink = coordinator,
        ).also { it.start() }
        getSystemService(NotificationManager::class.java).notify(
            NOTIFICATION_ID,
            buildNotification(logWriter.file.name),
        )
        handler.removeCallbacks(statusTask)
        handler.post(statusTask)
    }

    private fun stopLogging(reason: String) {
        stopCollectors(reason)
        broadcastStatus(logging = false)
        stopForeground(STOP_FOREGROUND_REMOVE)
        stopSelf()
    }

    private fun stopCollectors(reason: String) {
        handler.removeCallbacks(statusTask)
        fusedCollector?.stop()
        navigationMessageCollector?.stop()
        gnssCollector?.stop()
        imuCollector?.stop()
        fusedCollector = null
        navigationMessageCollector = null
        gnssCollector = null
        imuCollector = null
        liveCoordinator?.close()
        liveCoordinator = null
        writer?.close(reason)
        writer = null
        getSharedPreferences(PREFERENCES, MODE_PRIVATE).edit()
            .putBoolean(KEY_LOGGING, false)
            .apply()
    }

    private fun broadcastStatus(logging: Boolean) {
        val logWriter = writer
        val location = lastLocation
        val liveHealth = liveCoordinator?.health()
        val backendHealth = liveHealth?.backend
        val solution = backendHealth?.latestSolution
        val fgoSolution = backendHealth?.latestFgoSolution
        val intent = Intent(ACTION_STATUS)
            .setPackage(packageName)
            .putExtra(EXTRA_LOGGING, logging)
            .putExtra(EXTRA_SESSION_ID, logWriter?.sessionId.orEmpty())
            .putExtra(EXTRA_LOG_NAME, logWriter?.file?.name
                ?: getSharedPreferences(PREFERENCES, MODE_PRIVATE).getString(KEY_LOG_NAME, "").orEmpty())
            .putExtra(EXTRA_IMU_COUNT, imuCollector?.sampleCount?.get() ?: 0L)
            .putExtra(EXTRA_GNSS_EPOCH_COUNT, gnssCollector?.epochCount?.get() ?: 0L)
            .putExtra(EXTRA_GNSS_MEASUREMENT_COUNT, gnssCollector?.measurementCount?.get() ?: 0L)
            .putExtra(EXTRA_NAVIGATION_MESSAGE_COUNT, navigationMessageCollector?.messageCount?.get() ?: 0L)
            .putExtra(EXTRA_LOCATION_COUNT, fusedCollector?.locationCount?.get() ?: 0L)
            .putExtra(EXTRA_CLOCK_DISCONTINUITY, gnssCollector?.lastClockDiscontinuity?.get() ?: 0)
            .putExtra(EXTRA_STARTED_ELAPSED_NANOS, startedElapsedNanos)
            .putExtra(EXTRA_LIVE_QUEUED, liveHealth?.bus?.queuedEvents ?: 0)
            .putExtra(EXTRA_LIVE_PROCESSED, liveHealth?.bus?.processedEvents ?: 0L)
            .putExtra(EXTRA_LIVE_DROPPED, liveHealth?.bus?.droppedEvents ?: 0L)
            .putExtra(EXTRA_LIVE_CLEAN_GNSS_EPOCHS, liveHealth?.cleanGnssEpochs ?: 0L)
            .putExtra(EXTRA_LIVE_ACCEPTED_GNSS, liveHealth?.acceptedGnssObservations ?: 0L)
            .putExtra(EXTRA_LIVE_REJECTED_GNSS, liveHealth?.rejectedGnssObservations ?: 0L)
            .putExtra(EXTRA_LIVE_EPHEMERIS_MISSING, liveHealth?.unavailableEphemerisCount ?: 0L)
            .putExtra(EXTRA_LIVE_ESTIMATOR_STATE, liveHealth?.latestGnssState ?: "idle")
            .putExtra(EXTRA_COMPLETE_EPHEMERIDES, liveHealth?.completeEphemerides ?: 0)
            .putExtra(EXTRA_REJECTED_NAVIGATION_MESSAGES, liveHealth?.rejectedNavigationMessages ?: 0L)
            .putExtra(EXTRA_EPHEMERIS_REJECTION, liveHealth?.latestEphemerisRejection ?: "none")
            .putExtra(EXTRA_LIVE_BACKEND_STATE, backendHealth?.state ?: "waiting_for_clean_epoch")
            .putExtra(EXTRA_LIVE_WLS_UPDATES, backendHealth?.wlsUpdates ?: 0L)
            .putExtra(EXTRA_LIVE_WLS_FAILURES, backendHealth?.wlsFailures ?: 0L)
            .putExtra(EXTRA_LIVE_WLS_ENGINE, backendHealth?.wlsEngine ?: NativeGnssWls.readinessLabel)
            .putExtra(EXTRA_LIVE_NATIVE_WLS_FAILURES, backendHealth?.nativeWlsFailures ?: 0L)
            .putExtra(EXTRA_LIVE_INS_ENGINE, backendHealth?.insEngine ?: NativeInsMechanization.readinessLabel)
            .putExtra(
                EXTRA_LIVE_NATIVE_INS_PROPAGATIONS,
                backendHealth?.nativeInsPropagationUpdates ?: 0L,
            )
            .putExtra(EXTRA_LIVE_NATIVE_INS_FAILURES, backendHealth?.nativeInsFailures ?: 0L)
            .putExtra(EXTRA_LIVE_EKF_ENGINE, backendHealth?.ekfEngine ?: NativeGnssInsEkf.readinessLabel)
            .putExtra(
                EXTRA_LIVE_NATIVE_EKF_PROPAGATIONS,
                backendHealth?.nativeEkfPropagationUpdates ?: 0L,
            )
            .putExtra(EXTRA_LIVE_NATIVE_EKF_GNSS, backendHealth?.nativeEkfGnssUpdates ?: 0L)
            .putExtra(EXTRA_LIVE_NATIVE_EKF_FAILURES, backendHealth?.nativeEkfFailures ?: 0L)
            .putExtra(EXTRA_LIVE_EKF_PROPAGATIONS, backendHealth?.ekfPropagationUpdates ?: 0L)
            .putExtra(EXTRA_LIVE_EKF_GNSS_UPDATES, backendHealth?.ekfGnssUpdates ?: 0L)
            .putExtra(EXTRA_LIVE_STALE_GNSS, backendHealth?.staleGnssUpdates ?: 0L)
            .putExtra(EXTRA_LIVE_FGO_UPDATES, backendHealth?.fgoUpdates ?: 0L)
            .putExtra(EXTRA_LIVE_FGO_FAILURES, backendHealth?.fgoFailures ?: 0L)
            .putExtra(EXTRA_LIVE_FGO_ENGINE, backendHealth?.fgoEngine ?: NativeFixedLagSmoother.readinessLabel)
            .putExtra(
                EXTRA_LIVE_NATIVE_FGO_WINDOWS,
                backendHealth?.nativeFgoWindowUpdates ?: 0L,
            )
            .putExtra(EXTRA_LIVE_NATIVE_FGO_FAILURES, backendHealth?.nativeFgoFailures ?: 0L)
            .putExtra(EXTRA_LIVE_FGO_SKIPPED, backendHealth?.fgoSkippedBusy ?: 0L)
            .putExtra(EXTRA_LIVE_FGO_FEEDBACK_ACCEPTED, backendHealth?.fgoFeedbackAccepted ?: 0L)
            .putExtra(EXTRA_LIVE_FGO_FEEDBACK_REJECTED, backendHealth?.fgoFeedbackRejected ?: 0L)
            .putExtra(
                EXTRA_LIVE_FGO_FEEDBACK_REASON,
                backendHealth?.latestFgoFeedbackReason ?: "none",
            )
            .putExtra(
                EXTRA_LIVE_FGO_FEEDBACK_CORRECTION_METERS,
                backendHealth?.latestFgoFeedbackCorrectionMeters ?: 0.0,
            )
            .putExtra(EXTRA_LIVE_PAIRED_IMU, backendHealth?.pairedImuSamples ?: 0L)
            .putExtra(EXTRA_LIVE_UNPAIRED_IMU, backendHealth?.unpairedImuSamples ?: 0L)
            .putExtra(EXTRA_LIVE_IMU_PAIR_SKEW_MS, backendHealth?.maximumImuPairSkewMillis ?: 0.0)
            .putExtra(EXTRA_LIVE_HAS_SOLUTION, solution != null)
            .putExtra(EXTRA_LIVE_HAS_FGO_SOLUTION, fgoSolution != null)
        if (solution != null) {
            intent.putExtra(EXTRA_LIVE_SOLUTION_SOURCE, solution.source.name)
                .putExtra(EXTRA_LIVE_SOLUTION_ELAPSED_NANOS, solution.elapsedRealtimeNanos)
                .putExtra(EXTRA_LIVE_SOLUTION_LATITUDE, solution.latitudeDegrees)
                .putExtra(EXTRA_LIVE_SOLUTION_LONGITUDE, solution.longitudeDegrees)
                .putExtra(EXTRA_LIVE_SOLUTION_ALTITUDE, solution.altitudeMeters)
                .putExtra(EXTRA_LIVE_SOLUTION_HORIZONTAL_SIGMA, solution.horizontalSigmaMeters)
                .putExtra(EXTRA_LIVE_SOLUTION_VERTICAL_SIGMA, solution.verticalSigmaMeters)
                .putExtra(EXTRA_LIVE_SOLUTION_SPEED, solution.speedMetersPerSecond)
                .putExtra(EXTRA_LIVE_SOLUTION_LATENCY_MS, solution.solverLatencyMillis)
        }
        if (fgoSolution != null) {
            intent.putExtra(
                EXTRA_LIVE_FGO_SOLUTION_AGE_NANOS,
                maxOf(SystemClock.elapsedRealtimeNanos() - fgoSolution.elapsedRealtimeNanos, 0L),
            )
        }
        if (location != null) {
            intent.putExtra(EXTRA_HAS_LOCATION, true)
                .putExtra(EXTRA_LATITUDE, location.latitude)
                .putExtra(EXTRA_LONGITUDE, location.longitude)
                .putExtra(EXTRA_ALTITUDE, location.altitude)
                .putExtra(EXTRA_ACCURACY, location.accuracy)
        }
        sendBroadcast(intent)
    }

    private fun buildNotification(content: String): Notification {
        val openIntent = Intent(this, MainActivity::class.java)
        val openPending = PendingIntent.getActivity(
            this, 0, openIntent, PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT,
        )
        val stopIntent = Intent(this, LoggingService::class.java).setAction(ACTION_STOP)
        val stopPending = PendingIntent.getService(
            this, 1, stopIntent, PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT,
        )
        return Notification.Builder(this, CHANNEL_ID)
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setContentTitle("NavGraph recording")
            .setContentText(content)
            .setContentIntent(openPending)
            .setOngoing(true)
            .addAction(Notification.Action.Builder(null, "Stop", stopPending).build())
            .build()
    }

    private fun createNotificationChannel() {
        val channel = NotificationChannel(
            CHANNEL_ID,
            "Navigation logging",
            NotificationManager.IMPORTANCE_LOW,
        ).apply {
            description = "GNSS and inertial data collection status"
            setShowBadge(false)
        }
        getSystemService(NotificationManager::class.java).createNotificationChannel(channel)
    }

    companion object {
        const val ACTION_START = "com.example.navgraph.action.START"
        const val ACTION_STOP = "com.example.navgraph.action.STOP"
        const val ACTION_QUERY = "com.example.navgraph.action.QUERY"
        const val ACTION_STATUS = "com.example.navgraph.action.STATUS"
        const val EXTRA_LOGGING = "logging"
        const val EXTRA_SESSION_ID = "session_id"
        const val EXTRA_LOG_NAME = "log_name"
        const val EXTRA_IMU_COUNT = "imu_count"
        const val EXTRA_GNSS_EPOCH_COUNT = "gnss_epoch_count"
        const val EXTRA_GNSS_MEASUREMENT_COUNT = "gnss_measurement_count"
        const val EXTRA_NAVIGATION_MESSAGE_COUNT = "navigation_message_count"
        const val EXTRA_LOCATION_COUNT = "location_count"
        const val EXTRA_CLOCK_DISCONTINUITY = "clock_discontinuity"
        const val EXTRA_STARTED_ELAPSED_NANOS = "started_elapsed_nanos"
        const val EXTRA_HAS_LOCATION = "has_location"
        const val EXTRA_LATITUDE = "latitude"
        const val EXTRA_LONGITUDE = "longitude"
        const val EXTRA_ALTITUDE = "altitude"
        const val EXTRA_ACCURACY = "accuracy"
        const val EXTRA_LIVE_QUEUED = "live_queued"
        const val EXTRA_LIVE_PROCESSED = "live_processed"
        const val EXTRA_LIVE_DROPPED = "live_dropped"
        const val EXTRA_LIVE_CLEAN_GNSS_EPOCHS = "live_clean_gnss_epochs"
        const val EXTRA_LIVE_ACCEPTED_GNSS = "live_accepted_gnss"
        const val EXTRA_LIVE_REJECTED_GNSS = "live_rejected_gnss"
        const val EXTRA_LIVE_EPHEMERIS_MISSING = "live_ephemeris_missing"
        const val EXTRA_LIVE_ESTIMATOR_STATE = "live_estimator_state"
        const val EXTRA_COMPLETE_EPHEMERIDES = "complete_ephemerides"
        const val EXTRA_REJECTED_NAVIGATION_MESSAGES = "rejected_navigation_messages"
        const val EXTRA_EPHEMERIS_REJECTION = "ephemeris_rejection"
        const val EXTRA_LIVE_BACKEND_STATE = "live_backend_state"
        const val EXTRA_LIVE_WLS_UPDATES = "live_wls_updates"
        const val EXTRA_LIVE_WLS_FAILURES = "live_wls_failures"
        const val EXTRA_LIVE_WLS_ENGINE = "live_wls_engine"
        const val EXTRA_LIVE_NATIVE_WLS_FAILURES = "live_native_wls_failures"
        const val EXTRA_LIVE_INS_ENGINE = "live_ins_engine"
        const val EXTRA_LIVE_NATIVE_INS_PROPAGATIONS = "live_native_ins_propagations"
        const val EXTRA_LIVE_NATIVE_INS_FAILURES = "live_native_ins_failures"
        const val EXTRA_LIVE_EKF_ENGINE = "live_ekf_engine"
        const val EXTRA_LIVE_NATIVE_EKF_PROPAGATIONS = "live_native_ekf_propagations"
        const val EXTRA_LIVE_NATIVE_EKF_GNSS = "live_native_ekf_gnss"
        const val EXTRA_LIVE_NATIVE_EKF_FAILURES = "live_native_ekf_failures"
        const val EXTRA_LIVE_EKF_PROPAGATIONS = "live_ekf_propagations"
        const val EXTRA_LIVE_EKF_GNSS_UPDATES = "live_ekf_gnss_updates"
        const val EXTRA_LIVE_STALE_GNSS = "live_stale_gnss"
        const val EXTRA_LIVE_FGO_UPDATES = "live_fgo_updates"
        const val EXTRA_LIVE_FGO_FAILURES = "live_fgo_failures"
        const val EXTRA_LIVE_FGO_ENGINE = "live_fgo_engine"
        const val EXTRA_LIVE_NATIVE_FGO_WINDOWS = "live_native_fgo_windows"
        const val EXTRA_LIVE_NATIVE_FGO_FAILURES = "live_native_fgo_failures"
        const val EXTRA_LIVE_FGO_SKIPPED = "live_fgo_skipped"
        const val EXTRA_LIVE_FGO_FEEDBACK_ACCEPTED = "live_fgo_feedback_accepted"
        const val EXTRA_LIVE_FGO_FEEDBACK_REJECTED = "live_fgo_feedback_rejected"
        const val EXTRA_LIVE_FGO_FEEDBACK_REASON = "live_fgo_feedback_reason"
        const val EXTRA_LIVE_FGO_FEEDBACK_CORRECTION_METERS =
            "live_fgo_feedback_correction_meters"
        const val EXTRA_LIVE_PAIRED_IMU = "live_paired_imu"
        const val EXTRA_LIVE_UNPAIRED_IMU = "live_unpaired_imu"
        const val EXTRA_LIVE_IMU_PAIR_SKEW_MS = "live_imu_pair_skew_ms"
        const val EXTRA_LIVE_HAS_SOLUTION = "live_has_solution"
        const val EXTRA_LIVE_HAS_FGO_SOLUTION = "live_has_fgo_solution"
        const val EXTRA_LIVE_SOLUTION_SOURCE = "live_solution_source"
        const val EXTRA_LIVE_SOLUTION_ELAPSED_NANOS = "live_solution_elapsed_nanos"
        const val EXTRA_LIVE_SOLUTION_LATITUDE = "live_solution_latitude"
        const val EXTRA_LIVE_SOLUTION_LONGITUDE = "live_solution_longitude"
        const val EXTRA_LIVE_SOLUTION_ALTITUDE = "live_solution_altitude"
        const val EXTRA_LIVE_SOLUTION_HORIZONTAL_SIGMA = "live_solution_horizontal_sigma"
        const val EXTRA_LIVE_SOLUTION_VERTICAL_SIGMA = "live_solution_vertical_sigma"
        const val EXTRA_LIVE_SOLUTION_SPEED = "live_solution_speed"
        const val EXTRA_LIVE_SOLUTION_LATENCY_MS = "live_solution_latency_ms"
        const val EXTRA_LIVE_FGO_SOLUTION_AGE_NANOS = "live_fgo_solution_age_nanos"
        const val PREFERENCES = "logger_state"
        const val KEY_LOGGING = "logging"
        const val KEY_LOG_NAME = "log_name"
        private const val CHANNEL_ID = "navgraph_logging"
        private const val NOTIFICATION_ID = 2202
        private const val IMU_PERIOD_US = 10_000
    }
}
